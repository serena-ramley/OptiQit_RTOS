/* SECTION: Summary
 *  
 * Use LAC module to control linear actuator
 *
 * Functionality of relevant GPIOs:
 *   - GPIO16 - CLK+,
 *   - GPIO19 - CW+
 *
 * To do this test, you should connect GPIO16 to the CLK+ port of the TB6560 stepper driver
 * and connect GPIO19 to the CW+ port of the TB6560 stepper driver. 
 * Ensure ports CW- and CLK- are grounded. 
 *
 * An interrupt will be triggered when the linear actuation completes.
 */

void LAC_init(uint8_t CLK, uint8_t CW)
{
    pinMode(CLK, OUTPUT);

    pinMode(CW, OUTPUT);

    digitalWrite(CLK, LOW);

    digitalWrite(CW, LOW);
}

void LAC_move(uint8_t CLK)
{
    digitalWrite(CLK, HIGH);

    delayMicroseconds(100);

    digitalWrite(CLK, LOW);

    delayMicroseconds(100);

}
