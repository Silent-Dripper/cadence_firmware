#ifndef WRAPCOUNTER_h
#define WRAPCOUNTER_h

class wrapCounter
{
  public:
    wrapCounter(void);
    wrapCounter(int num_values);
    boolean increment(void);
    boolean decrement(void);
    void setCount(int count);
    void setNumValues(int num_values);
    void reset(void); 
    int value;
  private:
    int _num_values;
};

#endif
