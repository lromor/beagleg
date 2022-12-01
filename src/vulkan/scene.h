// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
// Copyright(c) Leonardo Romor <leonardo.romor@gmail.com>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation version 2.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://gnu.org/licenses/gpl-2.0.txt>

#ifndef __SCENE_H_
#define __SCENE_H_

#include <iostream>
#include <vulkan/vulkan.hpp>

#include "common/fd-mux.h"
#include "vulkan-core.h"
#include "entity.h"
#include "gamepad.h"
#include "camera.h"
#include "motion-queue.h"

// Given an initialized vulkan context
// perform rendering of the entities.
class VulkanScene {
public:
  ~VulkanScene()) 
  static VulkanScene *Create(FDMultiplexer *event_server);
  MotionQueue *GetSimFirmwareRenderingQueue();
private:
  class Impl;
  explicit VulkanScene(Impl *impl);
  std::unique_ptr<Impl> impl_;
};

#endif // __SCENE_H_
