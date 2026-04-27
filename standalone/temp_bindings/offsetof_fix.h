#pragma once
// Force Clang's builtin offsetof before any system headers
#undef offsetof
#define offsetof(s,m) __builtin_offsetof(s,m)
