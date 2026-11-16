// Logical node records only. Callers initialize positions before observing them.
#pragma once
#include "DetourNode.h"
#include <iostream>
#include <cstring>

inline void nodePoolContents(const dtNodePool& pool) {
    std::cout<<pool.getNodeCount()<<','<<pool.getMaxNodes()<<','<<pool.getHashSize()<<",[";
    for(int i=0;i<pool.getHashSize();++i){if(i)std::cout<<',';std::cout<<pool.getFirst(i);}std::cout<<"],[";
    for(int i=0;i<pool.getMaxNodes();++i){if(i)std::cout<<',';std::cout<<pool.getNext(i);}std::cout<<"],[";
    for(int i=0;i<pool.getNodeCount();++i) {
        if(i)std::cout<<',';const dtNode* n=pool.getNodeAtIdx(i+1);
        std::cout<<'['<<n->id<<','<<n->pidx<<','<<n->state<<','<<n->flags;
        for(float v:{n->pos[0],n->pos[1],n->pos[2],n->cost,n->total}) {unsigned bits;std::memcpy(&bits,&v,4);std::cout<<','<<bits;}
        std::cout<<']';
    }std::cout<<']';
}
