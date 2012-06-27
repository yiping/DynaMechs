/*
 *  wxNumericTextCtrl.h
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/19/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef __WX_NUMERIC_TEXT_CTRL__
#define __WX_NUMERIC_TEXT_CTRL__

#include "wx/textctrl.h"
#include "wx/wx.h"

class wxNumericTextCtrl : public wxTextCtrl {
public:
	wxNumericTextCtrl(wxWindow * parent,  wxWindowID id = wxID_ANY);
	double getValue();
	void setValue(double v);
private:
	double value;
	void OnChar(wxKeyEvent & event);
	
	DECLARE_EVENT_TABLE();
};



#endif