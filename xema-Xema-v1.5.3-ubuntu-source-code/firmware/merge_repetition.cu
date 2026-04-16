#include "merge_repetition.cuh"


__global__ void kernel_merge_pattern(unsigned char * const d_in_pattern,uint32_t img_height, uint32_t img_width,unsigned short * const d_out_merge_pattern)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * img_width + idx;

	if (idx < img_width && idy < img_height)
	{
  
		d_out_merge_pattern[offset] += d_in_pattern[offset];  

	}
}

__global__ void kernel_merge_pattern_16(unsigned short * const d_in_pattern,uint32_t img_height, uint32_t img_width,unsigned short * const d_out_merge_pattern)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * img_width + idx;

	if (idx < img_width && idy < img_height)
	{
  
		d_out_merge_pattern[offset] += d_in_pattern[offset];  

	}
}
 

__global__ void kernel_merge_brigntness_map(unsigned short * const merge_brightness,int repetition_count,uint32_t img_height, uint32_t img_width, unsigned char* brightness)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * img_width + idx;
 

	if (idx < img_width && idy < img_height)
	{ 
		brightness[offset] = 0.5 + (merge_brightness[offset]/repetition_count); 
  	 
	}
}

 