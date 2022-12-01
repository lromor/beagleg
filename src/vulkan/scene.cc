// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
// Copyright(c) 2019, NVIDIA CORPORATION. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// Modifications copyright (C) 2020 Leonardo Romor <leonardo.romor@gmail.com>
//
// This file contains the basic ingredients to render a basic wireframed scene.
// This simple scene allows you to add meshes and a freely "movable" camera.
#include <cstdio>
#include <glm/ext/quaternion_geometric.hpp>
#include <unistd.h>
#include <iostream>
#include <optional>
#include <X11/Xlib.h>
#include <vulkan/vulkan.hpp>
#include <numeric>
#include "xinput2.h"
#include "interface-manager.h"
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include "common/fd-mux.h"
#include "vulkan-core.h"
#include "scene.h"

#define FENCE_TIMEOUT 100000000

class VulkanScene::Impl {
public:
  typedef std::function<vk::Extent2D()> QueryExtentCallback;
  Impl(FDMultiplexer *event_server) : event_server_(event_server) {}
  ~Impl() {
    XCloseDisplay(display_);
  }
  bool Init();

private:
  FDMultiplexer *event_server_;
  Display *display_;
};

bool VulkanScene::Impl::Init() {
  const struct space::core::VkAppConfig config = {
    "BeagleG", "BeagleG Rendering Engine", {},
    { VK_KHR_SURFACE_EXTENSION_NAME,
      VK_KHR_XLIB_SURFACE_EXTENSION_NAME }};

  const unsigned kWidth = 1024;
  const unsigned kHeight = 768;

  XInitThreads();
  display_ = XOpenDisplay(NULL);
  if (display_ == NULL) {
    fprintf(stderr, "Couldn't open display.");
    return false;
  }

  int x11_fd = XConnectionNumber(display_);
  int screen = XDefaultScreen(display_);
  Window root_window = XRootWindow(display_, screen);
  Window window = XCreateSimpleWindow(
    display_, root_window, 0, 0, kWidth, kHeight, 0, 0, 0);
  XEvent event;

  // X11 Keyboard and mouse event utility class
  XInput2 xinput2(display_, window);
  xinput2.Init();

  XSelectInput(display_, window, ExposureMask);
  XMapWindow(display_, window);
  XSync(display_, False);

  XMaskEvent(display_, ExposureMask, &event);
  XSelectInput(display_, window, 0);

  auto get_window_extent = [&] () {
    XWindowAttributes attrs;
    XGetWindowAttributes(display_, window, &attrs);
    return vk::Extent2D(attrs.width, attrs.height);
  };

  // Initialize vulkan
  space::core::VkAppContext vk_ctx;

  if (auto ret = InitVulkan(config, display_, window)) {
    vk_ctx = std::move(ret.value());
  } else {
    fprintf(stderr, "Couldn't initialize vulkan.");
    return false;
  }
  // Whenever we get new input events, let's
  // call xinput2 which will trigger the related
  // callbacks and update the viewport.
  // event_server_->RunOnReadable(x11_fd, [&]() {
  //   while(XPending(display_)) {
  //     XNextEvent(display_, &event);
  //     xinput2.ReadEvents(event);
  //   }
  //   return true;
  // });
  return true;
}

VulkanScene *VulkanScene::Create(FDMultiplexer *event_server) {
  Impl *impl = new Impl(event_server);
  if (!impl->Init()) {
    delete impl;
    return NULL;
  }
  return new VulkanScene(impl);
}

MotionQueue *VulkanScene::GetSimFirmwareRenderingQueue() {
  return NULL;
}

VulkanScene::VulkanScene(Impl *impl) : impl_(impl) {}
VulkanScene::~VulkanScene() = default;
