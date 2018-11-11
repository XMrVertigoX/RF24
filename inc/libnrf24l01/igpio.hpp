#ifndef __IGPIO_HPP
#define __IGPIO_HPP

namespace libnrf24l01
{

class IGpio
{
public:
  virtual void set(bool enable = true) = 0;
};

} // namespace libnrf24l01

#endif /* __IGPIO_HPP */
