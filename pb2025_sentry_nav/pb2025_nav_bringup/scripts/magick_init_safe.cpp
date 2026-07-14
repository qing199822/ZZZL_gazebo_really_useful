// Fix for GraphicsMagick SIGSEGV in multi-threaded component_container
// Magick::InitializeMagick() is NOT thread-safe. This wrapper ensures
// it's called exactly once, from whichever thread reaches it first.
//
// Usage: LD_PRELOAD=/path/to/libmagick_init_safe.so <command>

#include <mutex>
#include <dlfcn.h>
#include <cstdio>

// C++ mangled symbol for Magick::InitializeMagick(char const*)
// We intercept the Magick++ C++ function

extern "C" {

// The actual InitializeMagick is a C++ function with mangling
// We use dlsym to get the real one and call_once to protect it

typedef void (*real_init_magick_t)(const char*);
static real_init_magick_t real_init_magick = nullptr;
static std::once_flag init_once;

// This wraps Magick::InitializeMagick with thread safety
// The mangled name depends on the ABI; we provide both common variants
}

// GCC/Clang mangling for Magick::InitializeMagick(char const*)
// _ZN6Magick16InitializeMagickEPKc

extern "C" void _ZN6Magick16InitializeMagickEPKc(const char* path) {
    if (!real_init_magick) {
        real_init_magick = (real_init_magick_t)dlsym(RTLD_NEXT, "_ZN6Magick16InitializeMagickEPKc");
    }
    std::call_once(init_once, [path]() {
        if (real_init_magick && real_init_magick != (real_init_magick_t)_ZN6Magick16InitializeMagickEPKc) {
            real_init_magick(path);
        }
    });
}

// Also wrap the MagickCore C API version: InitializeMagick(char const*)
extern "C" void InitializeMagick(const char* path) {
    typedef void (*real_func_t)(const char*);
    static real_func_t real_func = nullptr;
    static std::once_flag once;
    if (!real_func) {
        real_func = (real_func_t)dlsym(RTLD_NEXT, "InitializeMagick");
    }
    std::call_once(once, [path]() {
        if (real_func && real_func != (real_func_t)InitializeMagick) {
            real_func(path);
        }
    });
}

// Also wrap MagickCore Genesis: MagickCoreGenesis(char const*, MagickBooleanType)
extern "C" void MagickCoreGenesis(const char* path, int flag) {
    typedef void (*real_func_t)(const char*, int);
    static real_func_t real_func = nullptr;
    static std::once_flag once;
    if (!real_func) {
        real_func = (real_func_t)dlsym(RTLD_NEXT, "MagickCoreGenesis");
    }
    std::call_once(once, [path, flag]() {
        if (real_func && real_func != (real_func_t)MagickCoreGenesis) {
            real_func(path, flag);
        }
    });
}

// Constructor that runs at library load time to pre-initialize Magick safely
__attribute__((constructor))
static void preinit_magick() {
    // Initialize GM early in the main thread before any threads are spawned
    typedef void (*init_func_t)(const char*);
    init_func_t init_fn = (init_func_t)dlsym(RTLD_NEXT, "_ZN6Magick16InitializeMagickEPKc");
    if (init_fn) {
        init_fn(nullptr);
    }
}
