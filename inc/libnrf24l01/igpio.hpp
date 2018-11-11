#ifndef __IGPIO_HPP
#define __IGPIO_HPP

class IGpio
{
public:
  virtual void set(bool enable = true) = 0;
};

#endif /* __IGPIO_HPP */
