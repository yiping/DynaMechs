/*
 *  wxNumericTextCtrl.cpp
 *  DynaMechs
 *
 *  Created by Patrick Wensing on 6/19/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#include "wxNumericTextCtrl.h"
#include <ctype.h>
#include <iostream>
using namespace std;

wxNumericTextCtrl::wxNumericTextCtrl(wxWindow * parent,  wxWindowID id) : wxTextCtrl(parent,id) {


}


BEGIN_EVENT_TABLE(wxNumericTextCtrl, wxTextCtrl)
	EVT_CHAR  (wxNumericTextCtrl::OnChar)
END_EVENT_TABLE()


void wxNumericTextCtrl::OnChar(wxKeyEvent & event) {
	if (event.CmdDown() ) {
		event.Skip();
		return;
	}
	if (!isdigit(event.GetKeyCode())) {
		if (!(	 event.GetKeyCode() == 46 // period
			  || event.GetKeyCode() == 45 // dash
			  || event.GetKeyCode() == 8  // delete
			  || event.GetKeyCode() == 127 // backspace
			  || event.GetKeyCode() == 314 // left
			  || event.GetKeyCode() == 316 // right
		)) 
		{
			cout << "Invalid Key! " << event.GetKeyCode() << endl;
			return;
		}
		if (event.GetKeyCode() == 46) {
			if (!(this->GetValue().Find('.') == wxNOT_FOUND)) {
				return;
			}
		}
		if (event.GetKeyCode() == 45) {
			if (!(this->GetValue().Find('-') == wxNOT_FOUND)) {
				return;
			}
			else if (this->GetInsertionPoint() !=0)
			{
				return;
			}
			
		}
		
	}
	//cout << "Key Pos " << this->XYToPosition(event.GetX(),event.GetY()) << endl;
	//cout << "Key Y " << event.GetY() << endl;
	//cout << "Valid Key! " << event.KeyCode() << endl;
	event.Skip();
}