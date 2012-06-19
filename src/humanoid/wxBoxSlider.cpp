/*
 *  wxBoxSlider.cpp
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/19/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#include "wxBoxSlider.h"
#include "wx/event.h"
#include <iostream>
using namespace std;

wxBoxSlider::wxBoxSlider(wxWindow * parent, wxWindowID id,double minVal, double maxVal, int divs) 
	: wxPanel(parent,id) {

	sizer = new wxBoxSizer(wxHORIZONTAL);
		
	numericCtrl = new wxNumericTextCtrl((wxPanel*) this,wxBoxSlider::TEXT_ID);
	
	
	//int width, height;
	wxSize size = numericCtrl->GetSize();
	size.Scale(.5, 1);
	
	numericCtrl->SetInitialSize(size);
	//textCtrl->SetMaxSize(size);
	
	sizer->Add(numericCtrl,0,wxALL,4);
	
	slider = new wxSlider((wxPanel*)this ,wxBoxSlider::SLIDER_ID,0,0,divs);
	sizer->Add(slider,0,wxALL,4);
		
	this->SetSizer(sizer);
		
	min = minVal;
	max = maxVal;
	range = max-min;
	setValue(min);	
	divisions = divs;
}

BEGIN_EVENT_TABLE(wxBoxSlider, wxPanel)
	EVT_TEXT(  wxBoxSlider::TEXT_ID , wxBoxSlider::OnTextChange)
	EVT_COMMAND_SCROLL(wxBoxSlider::SLIDER_ID , wxBoxSlider::OnSliderChange)
	//EVT_BUTTON  (wxBoxSlider::BUTTON_ID,   wxBoxSlider::OnButtonPress)
END_EVENT_TABLE()


double wxBoxSlider::getValue() {
	return value;
}
void wxBoxSlider::setValue(double v) {
	value = v;
	UpdateSlider();
	UpdateText();
}

void wxBoxSlider::OnTextChange(wxCommandEvent & event)
{
	double v;
	wxString str = numericCtrl->GetValue();
	if(str.ToDouble(&value)) {
		UpdateSlider();
	}
}

void wxBoxSlider::OnSliderChange(wxScrollEvent & event)
{
	value = min + (range * slider->GetValue())/divisions;
	UpdateText();
}

void wxBoxSlider::UpdateText() {
	wxString mystring = wxString::Format(wxT("%.2lf"), value);
	numericCtrl->ChangeValue(mystring);
}

void wxBoxSlider::UpdateSlider() {
	int sliderVal = round((value-min)/range*divisions);
	if (sliderVal < 0) {
		sliderVal = 0;
	}
	else if (sliderVal > divisions)
	{
		sliderVal = divisions;
	}
	slider->SetValue(sliderVal);
}