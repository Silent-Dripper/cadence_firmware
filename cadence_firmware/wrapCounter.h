/*
  See the `.cpp` file for high level docs.
*/

#ifndef WRAPCOUNTER_h
#define WRAPCOUNTER_h

class wrapCounter {
public:
  wrapCounter(void);
  wrapCounter(int num_values);
  bool increment(void);
  bool decrement(void);
  void setCount(int count);
  void setNumValues(int num_values);
  void reset(void);
  int value;

private:
  int _num_values;
};

#endif
