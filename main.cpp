#include "config.h"
#include <iostream>
#include <sys/resource.h>
#include <unistd.h>
#include <sys/time.h>
#include <tbb/task_group.h>
#include <tbb.h>

Config g_config;

void flow();

void multithreadflow();

int main(int argc, char *argv[])
{
    struct rusage usage_before, usage_after;
    struct timeval start, end;
    gettimeofday(&start, NULL);
    g_config.parseArgs(argc, argv);
    getrusage(RUSAGE_SELF, &usage_before);
    if (g_config.num_threads == 1)
        flow();
    else
    {
        tbb::global_control control(tbb::global_control::max_allowed_parallelism, g_config.num_threads);
        // tbb::flow_control control(tbb::flow_control::max_allowed_parallelism, g_config.num_threads);
        multithreadflow();
    }
    getrusage(RUSAGE_SELF, &usage_after);
    gettimeofday(&end, NULL);
    std::cout << "Memory usage before function: " << usage_before.ru_maxrss << " KB" << std::endl;
    std::cout << "Memory usage after function: " << usage_after.ru_maxrss << " KB" << std::endl;
    // 计算程序运行时间
    double duration = (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec) / 1000000.0;
    // 输出运行时间
    std::cout << "Execution Time: " << duration << " seconds" << std::endl;

    return 0;
}