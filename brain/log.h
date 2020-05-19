#ifndef BRION_BRAIN_LOG_H
#define BRION_BRAIN_LOG_H

#include <iostream>
#include <stdexcept>

#define BRAIN_INFO std::cout << "[Brain][Info]"
#define BRAIN_WARN std::cout << "[Brain][Warning]"
#define BRAIN_ERROR std::cout << "[Brain][Error]"
#define BRAIN_THROW(message) \
{ \
    std::cerr << "[Brain][Critical]" << message << std::endl; \
    throw std::runtime_error(message); \
} \

#define BRAIN_THROW_IMPL(impl) \
{ \
    std::cerr << "[Brain][Critical] " << impl.what() << std::endl; \
    throw impl; \
} \

#endif
