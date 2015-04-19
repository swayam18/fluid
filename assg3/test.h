// generated by Fast Light User Interface Designer (fluid) version 1.0303

#ifndef test_h
#define test_h
#include "FL/Fl.H"
#include "FL/Fl_Double_Window.H"
#include "FL/Fl_Group.H"
#include "FL/Fl_Value_Slider.H"
#include "FL/Fl_Input.H"
#include "main.h"

class SliderInterface {
public:
  SliderInterface();
  float getSliderValue();
  float getMSliderValue();
  float getPSliderValue();
  float getTSliderValue();
  void setAirfoilCallback( bool * cb);
  Fl_Double_Window *window;
  Fl_Value_Slider *DensitySlider;
  Fl_Value_Slider *mSlider;
  Fl_Value_Slider *pSlider;
  Fl_Value_Slider *tSlider;
private:
  inline void cb_DensitySlider_i(Fl_Value_Slider*, void*);
  static void cb_DensitySlider(Fl_Value_Slider*, void*);

  inline void cb_mSlider_i(Fl_Value_Slider*, void*);
  static void cb_mSlider(Fl_Value_Slider*, void*);

  inline void cb_pSlider_i(Fl_Value_Slider*, void*);
  static void cb_pSlider(Fl_Value_Slider*, void*);

  inline void cb_tSlider_i(Fl_Value_Slider*, void*);
  static void cb_tSlider(Fl_Value_Slider*, void*);
  bool * cb;
public:
  void make_window();
};
#endif
// generated by Fast Light User Interface Designer (fluid) version 1.0303