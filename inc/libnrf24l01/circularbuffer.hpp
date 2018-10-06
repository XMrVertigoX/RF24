#ifndef __CIRCULARBUFFER_HPP
#define __CIRCULARBUFFER_HPP

#include <stdint.h>
#include <string.h>

template <typename TYPE>
class CircularBuffer
{
private:
  TYPE* elements = NULL;

  size_t maxElements;
  size_t numElements;
  size_t head;
  size_t tail;

  CircularBuffer(const CircularBuffer& other);            // Copy constructor
  CircularBuffer& operator=(const CircularBuffer& other); // Copy assignment operator
  CircularBuffer(CircularBuffer&& other);                 // Move constructor
  CircularBuffer& operator=(CircularBuffer&& other);      // Move assignment operator

public:
  CircularBuffer(size_t size);
  ~CircularBuffer();

  bool push_back(const TYPE& element);
  bool pop_front();

  TYPE& front();

  bool empty();
  bool full();
};

template <typename TYPE>
CircularBuffer<TYPE>::CircularBuffer(size_t size)
    : maxElements(size),
      numElements(0),
      tail(0),
      head(0)
{
  elements = new TYPE[maxElements];
}

template <typename TYPE>
CircularBuffer<TYPE>::~CircularBuffer()
{
  delete[] elements;
}

template <typename TYPE>
bool CircularBuffer<TYPE>::push_back(const TYPE& element)
{
  if (full())
  {
    return (false);
  }

  elements[tail] = element;

  tail = (tail + 1) % maxElements;
  numElements++;

  return (true);
}

template <typename TYPE>
bool CircularBuffer<TYPE>::pop_front()
{
  if (empty())
  {
    return (false);
  }

  // TODO: Clear element
  // elements[head] = nullptr;

  head = (head + 1) % maxElements;
  numElements--;

  return (true);
}

template <typename TYPE>
TYPE& CircularBuffer<TYPE>::front()
{
  // TODO: What if buffer is empty?
  return elements[head];
}

template <typename TYPE>
bool CircularBuffer<TYPE>::empty()
{
  return (!numElements);
}

template <typename TYPE>
bool CircularBuffer<TYPE>::full()
{
  return (maxElements == numElements);
}

#endif /* __CIRCULARBUFFER_HPP */
