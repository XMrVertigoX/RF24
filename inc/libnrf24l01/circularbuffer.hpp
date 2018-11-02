#ifndef __CIRCULARBUFFER_HPP
#define __CIRCULARBUFFER_HPP

#include <stdint.h>
#include <string.h>

template <typename TYPE>
class CircularBuffer
{
private:
  TYPE* elements = NULL;

  int maxElements;
  int numElements;
  int head;
  int tail;

  CircularBuffer(const CircularBuffer& other) = delete;            // Copy constructor
  CircularBuffer& operator=(const CircularBuffer& other) = delete; // Copy assignment operator
  CircularBuffer(CircularBuffer&& other) = delete;                 // Move constructor
  CircularBuffer& operator=(CircularBuffer&& other) = delete;      // Move assignment operator

public:
  CircularBuffer(int size);
  ~CircularBuffer();

  int push_back(const TYPE& element);
  int pop_front();

  TYPE& front();

  int empty();
  int full();
};

template <typename TYPE>
CircularBuffer<TYPE>::CircularBuffer(int size)
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
int CircularBuffer<TYPE>::push_back(const TYPE& element)
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
int CircularBuffer<TYPE>::pop_front()
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
int CircularBuffer<TYPE>::empty()
{
  return (!numElements);
}

template <typename TYPE>
int CircularBuffer<TYPE>::full()
{
  return (maxElements == numElements);
}

#endif /* __CIRCULARBUFFER_HPP */
