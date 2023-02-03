/**#include <miosix.h>
using namespace miosix;
int main()
{
    for(;;)
    {
        ledOn();
        Thread::sleep(1000);
        ledOff();
        Thread::sleep(1000);
    }
}**/

#include <cstdio>
#include "miosix.h"
#include "mxgui/display.h"
#include "mxgui/misc_inst.h"

using namespace miosix;
using namespace std;
using namespace mxgui;

int main()
{
	auto& display=DisplayManager::instance().getDisplay();
	{
		DrawingContext dc(display);
		dc.setFont(droid21);
		dc.write(Point(0,0), "Miosix OS");
	    dc.setFont(tahoma); 
		dc.write(Point(0,droid21.getHeight()), "MXGUI graphics library"); 	       
	}
    for(int i=0;; i++)
    {
	    {
		   DrawingContext dc(display); 
		  dc.write(Point(0,2*tahoma.getHeight()),"Try again");
	    }
	    Thread::sleep(1000); 
    }
}
