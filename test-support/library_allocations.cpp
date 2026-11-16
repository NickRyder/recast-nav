// Host-only macOS instrumentation for our benchmark child processes.
#include <cstdlib>
#include <cstdio>
#include <cstdint>
#include <pthread.h>
#include <unistd.h>
#include <execinfo.h>
#include <dlfcn.h>
#include <cstring>

static uint64_t calls, requested, frees;
static bool inside, stopped;
static bool enter() { if (!pthread_main_np() || inside || stopped) return false; inside=true;return true; }
// Optional attribution is separate from timing. Fixed host-only storage avoids
// recording the profiler's own allocations or perturbing the measured totals.
static constexpr unsigned stack_depth = 24, stack_slots = 8192;
struct StackCount { void* frames[stack_depth]; uint64_t calls, bytes; unsigned depth; };
static StackCount stacks[stack_slots];
static uint64_t dropped_calls, dropped_bytes;
static int trace_state;
static void trace(size_t size) {
    if (!trace_state) trace_state = getenv("RECAST_ALLOC_STACKS") ? 2 : 1;
    if (trace_state != 2) return;
    void* frames[stack_depth];
    int depth = backtrace(frames, stack_depth);
    if (depth <= 0) { ++dropped_calls; dropped_bytes += size; return; }
    uint64_t hash = 14695981039346656037ULL;
    for (int i = 0; i < depth; ++i) { hash ^= reinterpret_cast<uintptr_t>(frames[i]); hash *= 1099511628211ULL; }
    for (unsigned probe = 0; probe < stack_slots; ++probe) {
        auto& entry = stacks[(hash + probe) & (stack_slots - 1)];
        if (!entry.calls) {
            memcpy(entry.frames, frames, static_cast<size_t>(depth) * sizeof(void*));
            entry.depth = static_cast<unsigned>(depth);
        } else if (entry.depth != static_cast<unsigned>(depth) || memcmp(entry.frames, frames, static_cast<size_t>(depth) * sizeof(void*))) continue;
        ++entry.calls; entry.bytes += size; return;
    }
    ++dropped_calls; dropped_bytes += size;
}
static void record(bool outer,void* p,size_t size) { if(outer){if(p){++calls;requested+=size;trace(size);}inside=false;} }
extern "C" void* count_malloc(size_t n){bool outer=enter();void* p=malloc(n);record(outer,p,n);return p;}
extern "C" void* count_calloc(size_t n,size_t size){bool outer=enter();void* p=calloc(n,size);record(outer,p,n*size);return p;}
extern "C" void* count_realloc(void* old,size_t n){bool outer=enter();void* p=realloc(old,n);record(outer,p,n);return p;}
extern "C" int count_posix_memalign(void** p,size_t align,size_t n){bool outer=enter();int status=posix_memalign(p,align,n);record(outer,status==0?*p:nullptr,n);return status;}
extern "C" void* count_aligned_alloc(size_t align,size_t n){bool outer=enter();void* p=aligned_alloc(align,n);record(outer,p,n);return p;}
extern "C" void* count_valloc(size_t n){bool outer=enter();void* p=valloc(n);record(outer,p,n);return p;}
extern "C" void count_free(void* p){bool outer=enter();if(outer&&p)++frees;free(p);if(outer)inside=false;}
#define INTERPOSE(replacement,original) \
    __attribute__((used)) static const struct {const void* replacement;const void* original;} interpose_##original \
    __attribute__((section("__DATA,__interpose"))) = {reinterpret_cast<const void*>(&replacement),reinterpret_cast<const void*>(&original)}
INTERPOSE(count_malloc,malloc);
INTERPOSE(count_calloc,calloc);
INTERPOSE(count_realloc,realloc);
INTERPOSE(count_posix_memalign,posix_memalign);
INTERPOSE(count_aligned_alloc,aligned_alloc);
INTERPOSE(count_valloc,valloc);
INTERPOSE(count_free,free);
__attribute__((destructor)) static void report(){stopped=true;char buffer[256];int n=snprintf(buffer,sizeof(buffer),"ALLOCATIONS {\"calls\":%llu,\"requested_bytes\":%llu,\"frees\":%llu}\n",static_cast<unsigned long long>(calls),static_cast<unsigned long long>(requested),static_cast<unsigned long long>(frees));if(n>0)write(STDERR_FILENO,buffer,n);}

static void quoted(const char* value) {
    fputc('"', stderr);
    for (const unsigned char* p = reinterpret_cast<const unsigned char*>(value); *p; ++p) {
        if (*p == '"' || *p == '\\') { fputc('\\', stderr); fputc(*p, stderr); }
        else if (*p < 32) fprintf(stderr, "\\u%04x", *p);
        else fputc(*p, stderr);
    }
    fputc('"', stderr);
}
// Totals are stopped before symbol lookup/output, whose allocations are excluded.
__attribute__((destructor)) static void report_stacks() {
    stopped = true;
    if (trace_state != 2) return;
    fprintf(stderr, "ALLOCATION_STACKS {\"dropped_calls\":%llu,\"dropped_bytes\":%llu,\"stacks\":[", static_cast<unsigned long long>(dropped_calls), static_cast<unsigned long long>(dropped_bytes));
    bool first = true;
    for (const auto& entry : stacks) {
        if (!entry.calls) continue;
        if (!first) fputc(',', stderr); first = false;
        fprintf(stderr, "{\"calls\":%llu,\"requested_bytes\":%llu,\"frames\":[", static_cast<unsigned long long>(entry.calls), static_cast<unsigned long long>(entry.bytes));
        for (unsigned i = 0; i < entry.depth; ++i) {
            if (i) fputc(',', stderr);
            Dl_info info = {};
            bool found = dladdr(entry.frames[i], &info) != 0;
            const char* module = found && info.dli_fname ? info.dli_fname : "unknown";
            const char* slash = strrchr(module, '/'); if (slash) module = slash + 1;
            fprintf(stderr, "{\"module\":"); quoted(module);
            fprintf(stderr, ",\"pc\":\"%llx\",\"base\":\"%llx\"}", static_cast<unsigned long long>(reinterpret_cast<uintptr_t>(entry.frames[i])), static_cast<unsigned long long>(reinterpret_cast<uintptr_t>(info.dli_fbase)));
        }
        fprintf(stderr, "]}");
    }
    fprintf(stderr, "]}\n");
}
