#ifndef MERGE_REPETITION_CUDA_CUH
#define MERGE_REPETITION_CUDA_CUH
#pragma once 
#include <device_launch_parameters.h>
#include <device_functions.h>
#include <cuda_runtime.h>  
#include <cuda_texture_types.h>
#include <stdint.h>

__global__ void kernel_merge_brigntness_map(unsigned short * const merge_brightness,int repetition_count,uint32_t img_height, uint32_t img_width, unsigned char* brightness);

__global__ void kernel_merge_pattern(unsigned char * const d_in_pattern,uint32_t img_height, uint32_t img_width,unsigned short * const d_out_merge_pattern);

__global__ void kernel_merge_pattern_16(unsigned short * const d_in_pattern,uint32_t img_height, uint32_t img_width,unsigned short * const d_out_merge_pattern);
 
#endif