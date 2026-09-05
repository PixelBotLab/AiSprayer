#pragma once

#include <cstddef>

#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#elif defined(__APPLE__)
#include <pthread.h>
#endif

namespace orbbec_service {

inline void set_current_thread_name(const char* name) {
#if defined(__linux__)
    pthread_setname_np(pthread_self(), name);
#elif defined(__APPLE__)
    pthread_setname_np(name);
#else
    (void)name;
#endif
}

// RK3588 only: pin to the given CPU ids (A76 is 4-7). Other platforms no-op.
inline bool pin_current_thread_to_cpus(const int* cpus, std::size_t n) {
#if defined(HAS_RK3588) && defined(__linux__)
    cpu_set_t set;
    CPU_ZERO(&set);
    const long online = sysconf(_SC_NPROCESSORS_ONLN);
    bool any = false;
    for (std::size_t i = 0; i < n; ++i) {
        if (online > 0 && cpus[i] >= online) {
            continue;
        }
        CPU_SET(cpus[i], &set);
        any = true;
    }
    if (!any) {
        return false;
    }
    return pthread_setaffinity_np(pthread_self(), sizeof(set), &set) == 0;
#else
    (void)cpus;
    (void)n;
    return true;
#endif
}

// RK3588 的 4×A76。非 RK 平台 no-op 且返回 true。
inline bool pin_current_thread_to_rk3588_big_cores() {
    static const int kBigCores[] = {4, 5, 6, 7};
    return pin_current_thread_to_cpus(kBigCores, 4);
}

}  // namespace orbbec_service
