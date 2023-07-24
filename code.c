#include "mbed.h"
#include "MCP23017.h"
#include "WattBob_TextLCD.h"
#include "rtos.h"
#include "Math.h"

MCP23017 *par_port;
WattBob_TextLCD *lcd;
Serial pc(USBTX, USBRX);


DigitalOut engine_led(LED1);    // led for the engine state
DigitalOut brake_led(LED2);     // led for the brake state
DigitalOut monitor_led(LED3);   // led for the monitor
DigitalOut cruise_led(LED4);    // led for the cruise mode state


bool    PID = true;                 // if the PID is true : PID exponential / PID is false : PID lineair 
float   odom = 0;                    // initialization of the odometry
float   average_speed = 0;           // initialization of the average speed
float   current_speed = 0;           // initialization of the current speed
int     engine = 0;                  // initialization of the engine state
float   stock_speed[3];              // initialization of the stock of current speed
int     count1 = 0;                  // initialization of the count for stocked speed values
int     cruse = 0;                   // initialization of the cruise mode state
float   brake = 0;                   // initialization of the brake value
float   acceleration = 0;            // initialization of the acceleration value
int     cruise_timer = 0;            // initialization of the cruise timer 


//Semaphores to manage accessing the viriables 
Semaphore semaphore_inputs(1);             // controls the read and sent inputs
Semaphore semaphore_speed(1);                    // controls the calculated speed
Semaphore semaphore_average_speed(1);      // controls the calculated average speed

Thread thread1;
Thread thread2;
Thread thread3;
Thread thread4;
Thread thread5;
Thread thread6;
Thread thread7;
Thread thread8;

/*-------------------------------------------------------------------------
 *                          Read Break
 --------------------------------------------------------------------------
 * 
*/
void read_brake(void)
{
    while(true)
    {
        semaphore_inputs.wait();                    // takes the input semaphore 
        
        if (cruse == 0)                             // if cruse mode disable
        {
            if ( 1 == par_port -> read_bit(9))      // if the brake button is pressed
            {
                brake_led = 1;                      // led_2 = on
                brake += 1;                         // brake + 1
            }
            else                                    // if the button is not pressed
            {
                brake_led = 0;                      // led_2 = off                
                brake -= 1;                         // brake - 1
            }
            
                                                      // limit the brake to be between [0 ; 10] 
            
            if (brake > 10)                     
            {
                brake = 10;
            }
            else if (brake < 0)
            {
                brake = 0;
            }
        }
    
    semaphore_inputs.release();        // release input semaphore 
    Thread::wait(500);                 // rate of 2Hz
    }
}
/*-------------------------------------------------------------------------
 *                          Read Accelerator
 --------------------------------------------------------------------------
 * 
*/
void read_accelerator(void)
{
    while(true)
    {
        semaphore_inputs.wait();          // takes the input semaphore 
        
        if (cruse == 0)                   // if cruse mode disable
        {
            if (par_port -> read_bit(10) == 1)  // if the acceleration button is pressed
            {
                acceleration += 1;      // acceleration + 1
            }
            else
            {
                acceleration -= 1;      // acceleration - 1
            }
            
            // limit the acceleration to be between [0 ; 10]
            
            if (acceleration > 10)
            {
                acceleration = 10;
            }
            else if (acceleration < 0)
            {
                acceleration = 0;
            }
        }
    semaphore_inputs.release();    // release the input semaphore
    Thread::wait(500);             // rate of 2Hz
    }
}

/*-------------------------------------------------------------------------
 *                          Read Engine state
 --------------------------------------------------------------------------
 * 
*/
void read_engine(void)
{
    while(true)
    {
        semaphore_inputs.wait();                // takes the input semaphore 
        
        if (1 == par_port -> read_bit(8))       // if cruse mode disable
        {
            engine += 1;    // state engine takes +1
        }
        
        semaphore_inputs.release();             // Release the semaphore
                
        if (engine > 1)                         // if the engine state is above 1 the variable takes 0
        {
            engine = 0; 
        }
    
        if(engine == 1)                         // if the engine is ON 
        {
            engine_led = 1;   // led_1 = ON
        }
        else if(engine == 0)                     // if the engine is OFF 
        {
            engine_led = 0;   // led_1 = OFF
        }
    
    Thread::wait(500);                           // rate of 2Hz
    }
    
}


/*-------------------------------------------------------------------------
 *                          Read average speed
 --------------------------------------------------------------------------
 * 
*/
void read_average_speed()
{
    while(true)
    {
        semaphore_speed.wait();              // takes the semaphore speed
        semaphore_average_speed.wait();      // takes the sempagore averages speed
        
        int samples = 3;                     // number of samples
        int sum = 0;                         // sum is reset to 0
        
        for (int i=0;i < samples; i++)
        {
                sum = sum + current_speed;   // takes the 3 samples of the current speed
        }
            
        average_speed = sum / samples;       // and make an average of the sample
         average_speed = average_speed;
        
        semaphore_speed.release();           // release the semaphore speed
        semaphore_average_speed.release();   // release the semaphore average speed
        Thread::wait(150);                   // wait for
    
    }
}
/*-------------------------------------------------------------------------
 *                          monitor speed
 --------------------------------------------------------------------------
 * rate = 5Hz
*/
void monitor_speed()
{
    while(true)
    {
        semaphore_average_speed.wait();       // takes the average speed semaphore
        
        if (average_speed >= 40)              // it the average speed is above 40m/s
        {
            monitor_led = 1;        
            wait_ms(40);            // make the led blink
            monitor_led = 0;
        }

        semaphore_average_speed.release();    // release the sempahore
        Thread::wait(200);                    // rate of 5Hz
    }
 
            
    
}   

/*-------------------------------------------------------------------------
 *                          Read simulation
 --------------------------------------------------------------------------
 * Rate : 25Hz
*/
void simulation()
{
    while(true)
    {
        semaphore_speed.wait();                                  // takes the semaphore speed
        
        float time = 0.04;                                       // time value is created according to its rate
        
        if (cruse == 0 or ( cruse == 1 and PID == false))        // if the cruise mode is disable
        {
            current_speed = engine*(current_speed + ((acceleration - brake)*time));  // current speed equation
        }

                        
        if (current_speed < 0)                                    // limite the current speed to be under 0
        {
            current_speed = 0;    
    
        }
        else if (current_speed > 40)                              // limite the current speed to be over 40
        {
            current_speed = 40;    
    
        }

        
        odom = time*current_speed + odom;                         // equation of the odometry
        semaphore_speed.release();                                // release the semaphore
         
        stock_speed[count1] = current_speed;                      // takes a sample of the current speed for average speed
        count1 = count1 + 1;
        if(count1 == 3)                                           // once a reach 3 it write over the old values
        {
            count1 = 0;    
        }
    Thread::wait(40);                                             // rate of 25Hz
    }
}

/*-------------------------------------------------------------------------
 *                          Read LCD
 --------------------------------------------------------------------------
 * rate : 2Hz
*/
void display()
{
    while(true)
    {
        semaphore_inputs.wait();
        lcd->locate(0,0);                                                   // locate the beginning of the writing at the 1st line and 1st slot
        lcd->printf("speed:%.1f \n", average_speed);                        // write the frequency and the switch state
        lcd->locate(1,0);                                                   // locate the beginning of the writing at the 2st line and 1st slot
        lcd->printf("distance:%.1f\n",odom, acceleration);                  // write the value of the analogique signal 1 & 2
        semaphore_inputs.release();
        Thread::wait(500);                                                  // rate of 2Hz
    }
}

/*-------------------------------------------------------------------------
 *                          Crusing mode
 --------------------------------------------------------------------------
 *      Rate : 20Hz
*/
      
void crusing_mode()
{
    float tmp;                  // initialization of the tmp variable
    float delta_t = 0.05;       // initialization of the delta_t variable
    
    while(true)
    {
        semaphore_inputs.wait();
        semaphore_speed.wait(); 
        if (PID == true)        // if the PID is in true = 1st order PID
        {
            if (1 == par_port -> read_bit(11) and engine == 1) // if the Button is pressed
            {
                cruse += 1;                 // if the button is pressed the variable goes to 1
                
                if (cruse > 1)
                {
                    cruse = 0;              // if the button is pressed the variable was at 1 it goes back to 0
                }
                if (cruse == 0)             // if the variable is at 0 the led is switch off and the cruise mode diasabled
                {
                    cruse = 0;    
                    cruise_led = 0;
                    cruise_timer = 0;
                }
                if (cruse == 1 and cruise_timer == 0) // takes the value of the current speed to know where to start on the curve
                {
                cruise_led = 1;                      // switch on the led
                tmp = 2*log((current_speed/23 + 1)); // know the time on the curve
                cruise_timer = tmp/delta_t;          // transfom the time into discret time
                }
            }
            
            
            if (cruse == 1 and cruise_timer < 200)  // once the start time is known
            {
                
                current_speed = engine*23*(1-exp(-0.5*delta_t*cruise_timer)); // the current speed will follow the curves
                acceleration = engine*11*exp(-0.5*delta_t*cruise_timer);      // same for the acceleration
                cruise_timer += 1;                                            // increment the discrete time
            }
            else if (cruse == 1 and cruise_timer == 200)                      // once the speed reached the acceleration and brake takes 0
            {
                brake = 0.0;
                acceleration = 0.0;
                
            }
        }
        else // if the PID is in false = linair PID
        {
            if (1 == par_port -> read_bit(11) and engine == 1)  // read the cruise button
            {
                cruse += 1;     // enable cruise mode   
                cruise_led = 1; // switch on the light
                
                if (cruse > 1)
                {
                cruse = 0;    
                }
            
                if (cruse == 0)
                {
                    cruse = 0;    
                    cruise_led = 0;
                    cruise_timer = 0;
                }
                
                if(cruise_timer == 0) // if the cruise timer is at 0
                {
                    acceleration = (22 - current_speed)/10; // the acceleration predetermine lineair value
                    cruise_timer += 1;                      // the cruise timer is incremented
                }
            }
    
            else
            {
                if (cruse == 1)                     // if the acceleration has already been set
                {
                    if(cruise_timer > 164)              // if the timer is finished
                    {
                        acceleration = 0;               // set the acceleration at 0
                        
                    }
                    else                                // otherwise the acceleration keeps it value
                    {
                        cruise_timer += 1;
                    }
                }  
            }
        }
        
    semaphore_inputs.release();                         // release the semaphore
    semaphore_speed.release();
    Thread::wait(50);                                   // rate of the function 50
    }
}




/*-------------------------------------------------------------------------
 *                          MAIN
 --------------------------------------------------------------------------
 * 
*/

int main() {

    par_port = new MCP23017(p9, p10, 0x40);     // enable switch inputs
    lcd = new WattBob_TextLCD(par_port);        // enable LCD
    
    par_port->write_bit(1,BL_BIT);              //
    lcd->cls();                                 // clear the LCD
    lcd->locate(0,0);                           // Locate the writing at 0,0
    
    thread1.start(read_engine);                 // start the read engine tread
    thread2.start(read_brake);                  // start the read brake tread
    thread3.start(read_average_speed);          // start the read average speed tread
    thread4.start(read_accelerator);            // start the read accelerator tread
    thread5.start(monitor_speed);               // start the monitor speed tread
    thread6.start(display);                     // start the display tread
    thread7.start(crusing_mode);                // start the cruising mode tread
    thread8.start(simulation);                  // start the simulation tread
    
    while(1) 
    {
         /*
         read_engine();
         read_average_speed();
         read_accelerator();
         monitor_speed();
         display();
         crusing_mode();
         wait(1);
         simulation();
         */
    }
}