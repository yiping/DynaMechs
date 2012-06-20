/*
 *  wxBoxSlider.h
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/19/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef	__WX_BOX_SLIDER__
#define __WX_BOX_SLIDER__
#include "wx/wx.h"
#include "wx/control.h"
#include "wx/sizer.h"
#include "wx/slider.h"
#include "wxNumericTextCtrl.h"




class wxBoxSlider  : public wxPanel {
public:
	wxBoxSlider(wxWindow * parent, wxWindowID  id = wxID_ANY, double minVal = 0, double maxVal=1, int divs = 2);
	double getValue();
	void setValue(double v);
	
	enum
	{
		SLIDER_ID = wxID_HIGHEST + 1, // declares an id which will be used to call button
		TEXT_ID
	};
	
private:
	wxSlider * slider;
	wxNumericTextCtrl * numericCtrl;
	double value;
	double min;
	double max;
	double range;
	int divisions;
	void OnSliderChange(wxScrollEvent & event);
	void OnTextChange(wxCommandEvent & event);
	void UpdateSlider();
	void UpdateText();
	
	//void OnButtonPress(wxCommandEvent & event);
	wxBoxSizer * sizer;
	DECLARE_EVENT_TABLE()
};


#endif
