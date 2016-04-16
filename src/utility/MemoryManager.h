//
// Created by xiongyi on 4/3/16.
//

#ifndef CXY_LSDSLAM_MEMORYMANAGER_H
#define CXY_LSDSLAM_MEMORYMANAGER_H

#include <vector>
#include <memory>

namespace cxy
{
    template <class T>
    using ArrayPointer = std::unique_ptr<T[]>;

    /// vector to a unique_ptr
    /// unique_ptr point to a array : std::unique_ptr<int[]> my_array(new int[5]);
    template <class T>
    using ArrayPointer_Vector = std::vector<ArrayPointer<T> >;

    class MemoryManager
    {

    public:
        template <typename T>
        static ArrayPointer<T> ArrayPointer_Allocator(unsigned int size);

    };

    template <class T>
    ArrayPointer<T> MemoryManager::ArrayPointer_Allocator(unsigned int size)
    {
        return ArrayPointer<T>(new T[size]);
    }

}


#endif //CXY_LSDSLAM_MEMORYMANAGER_H
