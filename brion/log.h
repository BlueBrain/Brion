#ifndef BRION_BRION_LOG_H
#define BRION_BRION_LOG_H

#include <cassert>
#include <iostream>
#include <stdexcept>

#ifdef NDEBUG
#define BRION_DEBUG \
    if(false)       \
    std::cout
#else
#define BRION_DEBUG std::cout << "[Brion][Debug]"
#endif

#define BRION_INFO std::cout << "[Brion][Info]"
#define BRION_WARN std::cout << "[Brion][Warning]"
#define BRION_ERROR std::cout << "[Brion][Error]"

#define BRION_THROW(message) \
{ \
    std::cerr << "[Brion][Critical]" << message << std::endl; \
    throw std::runtime_error(message); \
} \

#define BRION_THROW_IMPL(impl) \
{ \
    throw impl; \
} \

//std::cerr << "[Brion][Critical] " << impl.what() << std::endl;

#define BRION_ASSERT(expr) assert(expr);

#define BRION_ASSERT_INFO(expr, message) \
{ \
    if(!expr) \
        BRION_ERROR << message << std::endl; \
    assert(expr); \
} \

#endif
