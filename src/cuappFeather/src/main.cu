#include "main.cuh"

__global__ void hello_kernel() {
    printf("Hello from CUDA kernel!\\n");
}

void TestCUDA()
{
    hello_kernel << <1, 1 >> > ();
    cudaDeviceSynchronize();
    std::cout << "CUDA finished\\n";
}
