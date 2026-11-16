#include <cstdlib>
#include <new>
int main(int argc,char** argv){if(argc!=2)return 1;int n=std::atoi(argv[1]);for(int i=0;i<n;++i){void* p=std::malloc(123);void* c=std::calloc(3,5);p=std::realloc(p,257);void* a=nullptr;if(posix_memalign(&a,64,128))return 2;void* b=aligned_alloc(64,128);if(!p||!c||!a||!b)return 3;std::free(p);std::free(c);std::free(a);std::free(b);void* extra=::operator new(17);::operator delete(extra);}return 0;}
