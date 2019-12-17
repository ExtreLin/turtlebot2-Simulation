#include"vulkanmodelrender.h"
#include<QtConcurrent>
#include <QVulkanFunctions>

static inline VkDeviceSize aligned(VkDeviceSize v, VkDeviceSize byteAlign)
{
    return (v + byteAlign - 1) & ~(byteAlign - 1);
}

VulkanModelRenderer::VulkanModelRenderer(VulkanWindow *w):
window_(w),cam_(),isUpdata_(false)
{
    //连接所有监视线程结束后刷新下一桢
   QObject::connect(&frameWatcher_, &QFutureWatcherBase::finished, [this] {
        if (framePending_) {
            framePending_ = false;
            //设置刷新
            window_->frameReady();
            window_->requestUpdate();
        }
    });     
}

void VulkanModelRenderer::preInitResources() 
{
  //判断是否支持多重采样
    const QVector<int> sampleCounts = window_->supportedSampleCounts();
    //如果支持，设置多重采样抗锯齿
    if (sampleCounts.contains(4)) {
        window_->setSampleCount(4);
    }
}

void VulkanModelRenderer::initResources()
{
    //获取实例和设备
    QVulkanInstance *inst = window_->vulkanInstance();
    VkDevice dev = window_->device();
    //获取对齐
    const VkPhysicalDeviceLimits *pdevLimits = &window_->physicalDeviceProperties()->limits;
    const VkDeviceSize uniAlign = pdevLimits->minUniformBufferOffsetAlignment;
    devFuncs_ = inst->deviceFunctions(dev);
    //获得顶点着色器中的uniform buffer 大小
    meshMaterial_.vertUniSize = aligned(2 * 64, uniAlign);
    //获得片元着色器中的uniform buffer 大小
    meshMaterial_.fragUniSize = aligned(24, uniAlign);
    //加载着色器文件
    if(!meshMaterial_.vs.isValid())
        meshMaterial_.vs.load(inst, dev, QStringLiteral("shader/vert.spv"));
    if(!meshMaterial_.gs.isValid())
        meshMaterial_.gs.load(inst, dev, QStringLiteral("shader/geom.spv"));
    if(!meshMaterial_.fs.isValid())
        meshMaterial_.fs.load(inst, dev, QStringLiteral("shader/frag.spv"));

    pipelinesFuture_ = QtConcurrent::run(this, &VulkanModelRenderer::createPipelines);
}

void VulkanModelRenderer::initSwapChainResources()
{
    const QSize sz = window_->swapChainImageSize();
    markViewProjDirty();
}

void VulkanModelRenderer::releaseSwapChainResources()
{
    //必须结束所有渲染线程才能释放
    frameWatcher_.waitForFinished();
    // Cannot count on the finished() signal being emitted before returning
    // from here.
    window_->frameReady();
    framePending_ = false;
}

void VulkanModelRenderer::releaseResources()
{
    //等到所有的渲染线程退出
    pipelinesFuture_.waitForFinished();
    VkDevice dev = window_->device();

    if (meshMaterial_.descSetLayout) {
        devFuncs_->vkDestroyDescriptorSetLayout(dev, meshMaterial_.descSetLayout, nullptr);
        meshMaterial_.descSetLayout = VK_NULL_HANDLE;
    }

    if (meshMaterial_.descPool) {
        devFuncs_->vkDestroyDescriptorPool(dev, meshMaterial_.descPool, nullptr);
        meshMaterial_.descPool = VK_NULL_HANDLE;
    }

    if (meshMaterial_.pipeline) {
        devFuncs_->vkDestroyPipeline(dev, meshMaterial_.pipeline, nullptr);
        meshMaterial_.pipeline = VK_NULL_HANDLE;
    }

    if (meshMaterial_.pipelineLayout) {
        devFuncs_->vkDestroyPipelineLayout(dev, meshMaterial_.pipelineLayout, nullptr);
        meshMaterial_.pipelineLayout = VK_NULL_HANDLE;
    }

    if (pipelineCache_) {
        devFuncs_->vkDestroyPipelineCache(dev, pipelineCache_, nullptr);
        pipelineCache_ = VK_NULL_HANDLE;
    }

    if (meshVertexBuf_) {
        devFuncs_->vkDestroyBuffer(dev, meshVertexBuf_, nullptr);
        meshVertexBuf_ = VK_NULL_HANDLE;
    }

    if (meshIndexBuf_) {
        devFuncs_->vkDestroyBuffer(dev, meshIndexBuf_, nullptr);
        meshIndexBuf_ = VK_NULL_HANDLE;
    }

    if (uniBuf_) {
        devFuncs_->vkDestroyBuffer(dev, uniBuf_, nullptr);
        uniBuf_ = VK_NULL_HANDLE;
    }

    if (bufMem_) {
        devFuncs_->vkFreeMemory(dev, bufMem_, nullptr);
        bufMem_ = VK_NULL_HANDLE;
    }

    if (meshMaterial_.vs.isValid()) {
        devFuncs_->vkDestroyShaderModule(dev, meshMaterial_.vs.data()->shaderModule, nullptr);
        meshMaterial_.vs.reset();
    }

    if (meshMaterial_.gs.isValid()) {
        devFuncs_->vkDestroyShaderModule(dev, meshMaterial_.gs.data()->shaderModule, nullptr);
        meshMaterial_.gs.reset();
    }

    if (meshMaterial_.fs.isValid()) {
        devFuncs_->vkDestroyShaderModule(dev, meshMaterial_.fs.data()->shaderModule, nullptr);
        meshMaterial_.fs.reset();
    }
}

void VulkanModelRenderer::startNextFrame()
{
    framePending_ = true;
    QFuture<void> future = QtConcurrent::run(this, &VulkanModelRenderer::buildFrame);
    frameWatcher_.setFuture(future);
}

void VulkanModelRenderer::buildFrame()
{
    //创建buffer,如这里是vertex buffer 和 uniform buffer
    ensureBuffers();
    //创建command buffer
    VkCommandBuffer cb = window_->currentCommandBuffer();
    const QSize sz = window_->swapChainImageSize();
    //设置背景
    VkClearColorValue clearColor = {{ 0.f,0.f,0.f, 1.0f }};
    VkClearDepthStencilValue clearDS = { 1, 0 };
    VkClearValue clearValues[3];
    memset(clearValues, 0, sizeof(clearValues));
    clearValues[0].color = clearValues[2].color = clearColor;
    clearValues[1].depthStencil = clearDS;

    VkRenderPassBeginInfo rpBeginInfo;
    memset(&rpBeginInfo, 0, sizeof(rpBeginInfo));
    rpBeginInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    rpBeginInfo.renderPass = window_->defaultRenderPass();
    rpBeginInfo.framebuffer = window_->currentFramebuffer();
    rpBeginInfo.renderArea.extent.width = sz.width();
    rpBeginInfo.renderArea.extent.height = sz.height();
    rpBeginInfo.clearValueCount = window_->sampleCountFlagBits() > VK_SAMPLE_COUNT_1_BIT ? 3 : 2;
    rpBeginInfo.pClearValues = clearValues;
    VkCommandBuffer cmdBuf = window_->currentCommandBuffer();
    devFuncs_->vkCmdBeginRenderPass(cmdBuf, &rpBeginInfo, VK_SUBPASS_CONTENTS_INLINE);

    VkViewport viewport = {
        0, 0,
        float(sz.width()), float(sz.height()),
        0, 1
    };
    devFuncs_->vkCmdSetViewport(cb, 0, 1, &viewport);

    VkRect2D scissor = {
        { 0, 0 },
        { uint32_t(sz.width()), uint32_t(sz.height()) }
    };
    devFuncs_->vkCmdSetScissor(cb, 0, 1, &scissor);
    //定义渲染数据绑定，可以将需要更新的数据在这里面写入
    buildDrawCallsForModel();
    devFuncs_->vkCmdEndRenderPass(cmdBuf);
}

void VulkanModelRenderer::ensureBuffers()
{
    //生成真正传入渲染的buffer
     if (!isUpdata_)
        return;
    isUpdata_ = false;

    VkDevice dev = window_->device();
    const int concurrentFrameCount = window_->concurrentFrameCount();
    //如果里面有数据先清空这些数据
    if(meshVertexBuf_){
        devFuncs_->vkDestroyBuffer(dev , meshVertexBuf_, nullptr);
        meshVertexBuf_ = VK_NULL_HANDLE;
    }
       
    if(meshIndexBuf_){
          devFuncs_->vkDestroyBuffer(dev , meshIndexBuf_, nullptr);
          meshIndexBuf_  = VK_NULL_HANDLE;
    }
  
    if (uniBuf_) {
        devFuncs_->vkDestroyBuffer(dev, uniBuf_, nullptr);
        uniBuf_ = VK_NULL_HANDLE;
    }

    if(bufMem_){
        devFuncs_->vkFreeMemory(dev, bufMem_, nullptr);
        bufMem_ = VK_NULL_HANDLE;
    }

    // 创建顶点buffer
    VkBufferCreateInfo bufInfo;
    memset(&bufInfo, 0, sizeof(bufInfo));
    bufInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    //我们只要顶点坐标，所以是3个点
    const int meshByteCount = mesh_.n_vertices() * 3 * sizeof(float);
    bufInfo.size = meshByteCount;
    bufInfo.usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
    VkResult err = devFuncs_->vkCreateBuffer(dev, &bufInfo, nullptr, &meshVertexBuf_);
    if (err != VK_SUCCESS)
        qFatal("Failed to create vertex buffer: %d", err);
    //创建应用这些顶点buffer 的请求实例
    VkMemoryRequirements meshVertMemReq;
    devFuncs_->vkGetBufferMemoryRequirements(dev, meshVertexBuf_, &meshVertMemReq);

    //开始生成index buffer
    const int indexByteCount =  mesh_.n_faces() * 3 * sizeof(uint);
    bufInfo.size = indexByteCount;
    bufInfo.usage = VK_BUFFER_USAGE_INDEX_BUFFER_BIT;
    err = devFuncs_->vkCreateBuffer(dev, &bufInfo, nullptr, &meshIndexBuf_);
     if (err != VK_SUCCESS)
        qFatal("Failed to create vertex buffer: %d", err);
    VkMemoryRequirements meshIndexMemReq;
    devFuncs_->vkGetBufferMemoryRequirements(dev, meshIndexBuf_, &meshIndexMemReq);

    //然后开始生成uniform buffer 
    bufInfo.size = (meshMaterial_.vertUniSize + meshMaterial_.fragUniSize) * concurrentFrameCount;
    bufInfo.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
    err = devFuncs_->vkCreateBuffer(dev, &bufInfo, nullptr, &uniBuf_);
    if (err != VK_SUCCESS)
        qFatal("Failed to create uniform buffer: %d", err);
    //创建应用这些uniform buffer 所需要的请求实例
    VkMemoryRequirements uniMemReq;
    devFuncs_->vkGetBufferMemoryRequirements(dev, uniBuf_, &uniMemReq);

    //这里要将顶点buffer\index buffer 和uniform buffer 放在一个显存块上，所以要计算unifrom buffer 在 vertex buffer 后的offset
    int indexStartOffset =  aligned(meshVertMemReq.size,meshIndexMemReq.alignment);
    meshMaterial_.uniMemStartOffset = aligned(indexStartOffset +  meshIndexMemReq.size,  uniMemReq.alignment);
    //在GPU上申请一块可以容纳顶点buffer 和 uniform bufferr大小的显存块
    VkMemoryAllocateInfo memAllocInfo = {
        VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,
        nullptr,
        meshMaterial_.uniMemStartOffset + uniMemReq.size,
        window_->hostVisibleMemoryIndex()
    };
    err = devFuncs_->vkAllocateMemory(dev, &memAllocInfo, nullptr, &bufMem_);
    if (err != VK_SUCCESS)
        qFatal("Failed to allocate memory: %d", err);
    //将顶点buffer 信息绑定到这块显存块上
    err = devFuncs_->vkBindBufferMemory(dev, meshVertexBuf_, bufMem_, 0);
    if (err != VK_SUCCESS)
        qFatal("Failed to bind vertex buffer memory: %d", err);

    //将索引buffer 信息绑定到这块显存块上
    err = devFuncs_->vkBindBufferMemory(dev, meshIndexBuf_ , bufMem_, indexStartOffset);
    if (err != VK_SUCCESS)
        qFatal("Failed to bind vertex buffer memory: %d", err);
        
    //在同一块显存块上的顶点buffer 后面绑定 uniform buffer
    err = devFuncs_->vkBindBufferMemory(dev, uniBuf_, bufMem_, meshMaterial_.uniMemStartOffset);
    if (err != VK_SUCCESS)
        qFatal("Failed to bind uniform buffer memory: %d", err);

    // 开始向显存块上拷贝数据，所以将显卡块map
    //首先map vertex buffer 所以只到uniMemStartOffset 过
    quint8 *p;
    err = devFuncs_->vkMapMemory(dev, bufMem_, 0, indexStartOffset, 0, reinterpret_cast<void **>(&p));
    if (err != VK_SUCCESS)
        qFatal("Failed to map memory: %d", err);
    //生成顶点数据
    std::vector<float>   vertexData(mesh_.n_vertices()*3);
    for(int i=0;i <mesh_.n_vertices();++i)
    {
        vertexData[i*3 ] = mesh_.point(mesh_.vertex_handle(i))[0]/1000.0f;
        vertexData[i*3 + 1] = mesh_.point(mesh_.vertex_handle(i))[1]/1000.0f;
        vertexData[i*3 + 2] = mesh_.point(mesh_.vertex_handle(i))[2]/1000.0f;
    }
    //拷贝，并且unmap
    memcpy(p, vertexData.data(), meshByteCount);
    devFuncs_->vkUnmapMemory(dev, bufMem_);
    //生成索引数据
     err = devFuncs_->vkMapMemory(dev, bufMem_, indexStartOffset, meshMaterial_.uniMemStartOffset , 0, reinterpret_cast<void **>(&p));
    if (err != VK_SUCCESS)
        qFatal("Failed to map memory: %d", err);
    //生成顶点数据
    std::vector<uint>   indexData(mesh_.n_faces()*3);
    for(int i=0;i <mesh_.n_faces();++i)
    {
        auto fn = mesh_.face_handle(i);
        int count =0;
        for(auto fv_iter = mesh_.fv_begin(fn);fv_iter!= mesh_.fv_end(fn);fv_iter++,count ++)
        {
            indexData[i*3  + count] = uint(fv_iter->idx());
        }
    }
    //拷贝，并且unmap
    memcpy(p, indexData.data(), indexByteCount);
    devFuncs_->vkUnmapMemory(dev, bufMem_);

    // 然后将开始就生成的描述池中写入uniform buffer 描述实例信息
    VkDescriptorBufferInfo vertUni = { uniBuf_, 0, meshMaterial_.vertUniSize };
    VkDescriptorBufferInfo fragUni = { uniBuf_, meshMaterial_.vertUniSize, meshMaterial_.fragUniSize };

    VkWriteDescriptorSet descWrite[2];
    memset(descWrite, 0, sizeof(descWrite));
    //先生成写入顶点 shader 中 的 uniform buffer 信息的实例
    descWrite[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descWrite[0].dstSet = meshMaterial_.descSet;
    descWrite[0].dstBinding = 0;
    descWrite[0].descriptorCount = 1;
    descWrite[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC;
    descWrite[0].pBufferInfo = &vertUni;
    //再生成写入 片段 shader 中 的 uniform buffer 信息的实例
    descWrite[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descWrite[1].dstSet = meshMaterial_.descSet;
    descWrite[1].dstBinding = 1;
    descWrite[1].descriptorCount = 1;
    descWrite[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC;
    descWrite[1].pBufferInfo = &fragUni;
    //最后一把更新写入
    devFuncs_->vkUpdateDescriptorSets(dev, 2, descWrite, 0, nullptr);
}

void VulkanModelRenderer::buildDrawCallsForModel()
{
    if(!mesh_.n_vertices())
        return;
    VkDevice dev = window_->device();
    VkCommandBuffer cb = window_->currentCommandBuffer();

    devFuncs_->vkCmdBindPipeline(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, meshMaterial_.pipeline);

    VkDeviceSize vbOffset = 0;
    devFuncs_->vkCmdBindVertexBuffers(cb, 0, 1, &meshVertexBuf_, &vbOffset);
    devFuncs_->vkCmdBindIndexBuffer(cb, meshIndexBuf_, 0,VK_INDEX_TYPE_UINT32);

    // Now provide offsets so that the two dynamic buffers point to the
    // beginning of the vertex and fragment uniform data for the current frame.
    uint32_t frameUniOffset = window_->currentFrame() * (meshMaterial_.vertUniSize + meshMaterial_.fragUniSize);
    uint32_t frameUniOffsets[] = { frameUniOffset, frameUniOffset };
    devFuncs_->vkCmdBindDescriptorSets(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, meshMaterial_.pipelineLayout, 0, 1,
                                        &meshMaterial_.descSet, 2, frameUniOffsets);

    if (vpDirty_) {
        --vpDirty_;
        QMatrix4x4 vp, model;
        QMatrix3x3 modelNormal;
        QVector3D eyePos;
        getMatrices(&vp, &model, &eyePos);

        // Map the uniform data for the current frame, ignore the geometry data at
        // the beginning and the uniforms for other frames.
        quint8 *p;
        VkResult err = devFuncs_->vkMapMemory(dev, bufMem_,
                                               meshMaterial_.uniMemStartOffset + frameUniOffset,
                                               meshMaterial_.vertUniSize + meshMaterial_.fragUniSize,
                                               0, reinterpret_cast<void **>(&p));
        if (err != VK_SUCCESS)
            qFatal("Failed to map memory: %d", err);

        // Vertex shader uniforms
        memcpy(p, vp.constData(), 64);
        memcpy(p + 64, model.constData(), 64);
        // Fragment shader uniforms
        float constColorData[] = { 0.75,0.75,0.75 };
        //由于对齐问题，所以下一个uniform buffer 需要在规定的对齐数据下写
        memcpy(p + meshMaterial_.vertUniSize, constColorData, 12);
        float eyePositionData[] = { eyePos.x(),eyePos.y(),eyePos.z() };
        memcpy(p + meshMaterial_ .vertUniSize + 12, eyePositionData, 12 );
        devFuncs_->vkUnmapMemory(dev, bufMem_);
    }
    devFuncs_->vkCmdDrawIndexed(cb, mesh_.n_faces(),1, 0, 0,0);
}

void VulkanModelRenderer::createPipelines()
{
    VkDevice dev = window_->device();
    //生成Pipelines 缓存生成信息
    VkPipelineCacheCreateInfo pipelineCacheInfo;
    memset(&pipelineCacheInfo, 0, sizeof(pipelineCacheInfo));
    pipelineCacheInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_CACHE_CREATE_INFO;
    VkResult err = devFuncs_->vkCreatePipelineCache(dev, &pipelineCacheInfo, nullptr, &pipelineCache_);
    if (err != VK_SUCCESS)
        qFatal("Failed to create pipeline cache: %d", err);

    // 顶点shader绑定描述信息 绑定信息和VkVertexInputAttributeDescription 和 vkCmdBindVertexBuffers中的绑定一致
    VkVertexInputBindingDescription vertexBindingDesc[] = {
        {
            0, // binding
            3 * sizeof(float),
            VK_VERTEX_INPUT_RATE_VERTEX
        }
    };
    //顶点shader 输入信息，根据shader 里面的location一致
    VkVertexInputAttributeDescription vertexAttrDesc[] = {
        { // position
            0, // location
            0, // binding
            VK_FORMAT_R32G32B32_SFLOAT,
            0 // offset
        }
    };
    //生成Pipeline中的顶点输入的创建信息，后续生成 pipeline的时候i会用到
    VkPipelineVertexInputStateCreateInfo vertexInputInfo;
    vertexInputInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
    vertexInputInfo.pNext = nullptr;
    vertexInputInfo.flags = 0;
    vertexInputInfo.vertexBindingDescriptionCount = 1;
    vertexInputInfo.pVertexBindingDescriptions = vertexBindingDesc;
    vertexInputInfo.vertexAttributeDescriptionCount = 1;
    vertexInputInfo.pVertexAttributeDescriptions = vertexAttrDesc;

  // 生成描述池（基本上可以理解用于描述 uniform buffer 怎么布局和绑定的 也可以用于 ssbo）
    VkDescriptorPoolSize descPoolSizes[] = {
        { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, 2 }
    };
    VkDescriptorPoolCreateInfo descPoolInfo;
    memset(&descPoolInfo, 0, sizeof(descPoolInfo));
    descPoolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    descPoolInfo.maxSets = 1; // a single set is enough due to the dynamic uniform buffer
    descPoolInfo.poolSizeCount = sizeof(descPoolSizes) / sizeof(descPoolSizes[0]);
    descPoolInfo.pPoolSizes = descPoolSizes;
    err = devFuncs_->vkCreateDescriptorPool(dev, &descPoolInfo, nullptr, &meshMaterial_.descPool);
    if (err != VK_SUCCESS)
        qFatal("Failed to create descriptor pool: %d", err);

    //布局这些uniform buffer  ，首先要布局各个绑定
    VkDescriptorSetLayoutBinding layoutBindings[] =
    {
        {
            0, // binding
            VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, // 也可以是ssbo VK_DESCRIPTOR_TYPE_STORAGE_BUFFER
            1, // descriptorCount
            VK_SHADER_STAGE_VERTEX_BIT,
            nullptr
        },
        {
            1,
            VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC,
            1,
            VK_SHADER_STAGE_FRAGMENT_BIT,
            nullptr
        }
    };

    //创建描述布局生成信息
    VkDescriptorSetLayoutCreateInfo descLayoutInfo = {
        VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO,
        nullptr,
        0,
        sizeof(layoutBindings) / sizeof(layoutBindings[0]),
        layoutBindings
    };
    //生成描述布局
    err = devFuncs_->vkCreateDescriptorSetLayout(dev, &descLayoutInfo, nullptr, &meshMaterial_.descSetLayout);
    if (err != VK_SUCCESS)
        qFatal("Failed to create descriptor set layout: %d", err);
    //生成这些描述的设置
    VkDescriptorSetAllocateInfo descSetAllocInfo = {
        VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO,
        nullptr,
        meshMaterial_.descPool,
        1,
        &meshMaterial_.descSetLayout
    };
    err = devFuncs_->vkAllocateDescriptorSets(dev, &descSetAllocInfo, &meshMaterial_.descSet);
    if (err != VK_SUCCESS)
        qFatal("Failed to allocate descriptor set: %d", err);

    // 生成图形的pipeline
    //还是老套路，首先创建pipeline布局生成信息
    VkPipelineLayoutCreateInfo pipelineLayoutInfo;
    memset(&pipelineLayoutInfo, 0, sizeof(pipelineLayoutInfo));
    pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = &meshMaterial_.descSetLayout;
    //生成pipeline布局
    err = devFuncs_->vkCreatePipelineLayout(dev, &pipelineLayoutInfo, nullptr, &meshMaterial_.pipelineLayout);
    if (err != VK_SUCCESS)
        qFatal("Failed to create pipeline layout: %d", err);
    //生成创建 图形pipeline 创建信息
    VkGraphicsPipelineCreateInfo pipelineInfo;
    memset(&pipelineInfo, 0, sizeof(pipelineInfo));
    pipelineInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
    //创建shader
    VkPipelineShaderStageCreateInfo shaderStages[3] = {
        {
            VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
            nullptr,
            0,
            VK_SHADER_STAGE_VERTEX_BIT,
            meshMaterial_.vs.data()->shaderModule,
            "main",
            nullptr
        },
        {
            VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
            nullptr,
            0,
            VK_SHADER_STAGE_GEOMETRY_BIT,
            meshMaterial_.gs.data()->shaderModule,
            "main",
            nullptr
        },
        {
            VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
            nullptr,
            0,
            VK_SHADER_STAGE_FRAGMENT_BIT,
            meshMaterial_.fs.data()->shaderModule,
            "main",
            nullptr
        }
    };
    //将shader 加入 图形pipeline 创建信息中
    pipelineInfo.stageCount = 3;
    pipelineInfo.pStages = shaderStages;
    //将最开始生成的顶点输入信息加入 图形pipeline 中
    pipelineInfo.pVertexInputState = &vertexInputInfo;
    //一些其他设置
    VkPipelineInputAssemblyStateCreateInfo ia;
    memset(&ia, 0, sizeof(ia));
    ia.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
    ia.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    pipelineInfo.pInputAssemblyState = &ia;

    VkPipelineViewportStateCreateInfo vp;
    memset(&vp, 0, sizeof(vp));
    vp.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
    vp.viewportCount = 1;
    vp.scissorCount = 1;
    pipelineInfo.pViewportState = &vp;

    VkPipelineRasterizationStateCreateInfo rs;
    memset(&rs, 0, sizeof(rs));
    rs.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
    rs.polygonMode = VK_POLYGON_MODE_FILL;
    rs.cullMode = VK_CULL_MODE_NONE;
    rs.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    rs.lineWidth = 1.0f;
    pipelineInfo.pRasterizationState = &rs;

    VkPipelineMultisampleStateCreateInfo ms;
    memset(&ms, 0, sizeof(ms));
    ms.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
    ms.rasterizationSamples = window_->sampleCountFlagBits();
    pipelineInfo.pMultisampleState = &ms;

    VkPipelineDepthStencilStateCreateInfo ds;
    memset(&ds, 0, sizeof(ds));
    ds.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
    ds.depthTestEnable = VK_TRUE;
    ds.depthWriteEnable = VK_TRUE;
    ds.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
    pipelineInfo.pDepthStencilState = &ds;

    VkPipelineColorBlendStateCreateInfo cb;
    memset(&cb, 0, sizeof(cb));
    cb.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
    VkPipelineColorBlendAttachmentState att;
    memset(&att, 0, sizeof(att));
    att.colorWriteMask = 0xF;
    cb.attachmentCount = 1;
    cb.pAttachments = &att;
    pipelineInfo.pColorBlendState = &cb;

    VkDynamicState dynEnable[] = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
    VkPipelineDynamicStateCreateInfo dyn;
    memset(&dyn, 0, sizeof(dyn));
    dyn.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
    dyn.dynamicStateCount = sizeof(dynEnable) / sizeof(VkDynamicState);
    dyn.pDynamicStates = dynEnable;
    pipelineInfo.pDynamicState = &dyn;
    //最后将生成描述的布局信息加入 图形pipeline 中
    pipelineInfo.layout = meshMaterial_.pipelineLayout;
    pipelineInfo.renderPass = window_->defaultRenderPass();
    //最后生成  图形pipeline ，类似 opengl 中将shader 编译好，然后shader 的顶点绑定信息和unifrom 怎么输入的 都定义好，
    //最后使用的时候只要将实际的 vertex buffer 绑定到相同的位置就是了
    //其实就是定义顶点数据改怎么用于渲染，虽然这个时候顶点数据还没生产，但是定义了规则
    err = devFuncs_->vkCreateGraphicsPipelines(dev, pipelineCache_, 1, &pipelineInfo, nullptr, &meshMaterial_.pipeline);
    if (err != VK_SUCCESS)
        qFatal("Failed to create graphics pipeline: %d", err);
}
void VulkanModelRenderer::setMesh(const TriMesh& mesh)
{
    mesh_ = mesh;
    isUpdata_ = true;
    markViewProjDirty();
}

void VulkanModelRenderer::getMatrices(QMatrix4x4 *mvp, QMatrix4x4 *model, QVector3D *eyePos)
{
    model->setToIdentity();
    //由于opengl的裁剪面的坐标系和vulkan不一致，所以先要在做投影矩阵的时候先给
    //一个修正两者不一致的矩阵
    cam_.projection(window_->clipCorrectionMatrix());
    //model->rotate(rotation, 1, 1, 0);
    QMatrix4x4 view = cam_.qview_;
    *model = view*(*model);
    *mvp = cam_.qpoj_* view;

    cam_.get_eye(*eyePos);
}

