# ENGS22 Blackbox Lab
## Jake Epstein

Arrange the diagram according to figure A

define the following possible types of classifications of data type `classification`

```
time_invariant
RC_series
RC_parallel
RL_series
RL_parallel
second-order system
```

## Linearity Test

Assign variables

```
V = some I_DC_2
X = some I_DC
alpha A = some multipler
V_box(i_in) is v_out as a function of i_in as seen in figure A
i_in is some DC current
```
Linearity test algorithm

```
classification int main(void) {

	if(V_box(A*x) == A*V_box(x)) {
		if(V_box(x+y) = V_box(x+y)) {
			linear == true;
		} else {
			linear == false;
			return time_invariant
		}
	} else {
		linear == false;
		return time_invariant
	}

	if(linear == true) {
	//	setup oscilloscope (see figure B);
		resonance_test();
		if(resonance_test == false) {
			firstorder_test();
		} else {
			secondorder_test();
		}
	}
}

classification firstorder_test() {
	
	//attach multimeter to black-box (figure c)
	if(lim(t->infinity, R(t)) = OL) {
		//as t approaches infinity, capacitor charges and becomes a break
		return RC_series;
	}
	/*reattach circuit in figure B;
	 *Ensure clean sine output with no DC offset, A & B read zero when no input;
	 *set function generator 1 kHz and 1 V_p-p;
	 *display box channels, trigger off channgel A, set both to AC coupling
	 *adjust time divisions to display the largest full wave of channel A
	 */

	while(seperation between channel A & channel B is bad) {
		adjust frequency;
	}

	// V_A = voltage across test circuit
	// V_B = current through test circuit

	if(current leads voltage) {
		return RC-parallel;
	} else {
		// inductor testing 
		set frequency to 0;
		// if same waveform
		if(mag(V_A) == mag(V_B)) {
			//set frequency to high value (20 kHz)
			if((in_phase() == true) && (mag(V_A) > mag(V_B))) {
				return RL_parallel;
			}
		} else if((mag(V_A) > mag(V_B) && (in_phase() == true)) 
			//set frequency to high value to 20 kHZ
			//consider 50 ohm internal resistor in load mode when evaluating if V_A == input
			if (V_B ==0 && V_A == input) {
				return RL_series;
			}
		}
		find_values;
}

bool resonance_test() {
	//Test for resonance to determine if the blackbox is a first or second order system,
	//There are 2 possible time constants for a first order system tau = RC and tau = 1/LR
	//drive the circuit with AC, find steady state.
	for(tau = 1; tau < 3; tau ++) {
		//if tau matches known percentage of steady state for 3 time constants
		return true;
	}
	return false; 
}

void find_values() {

	if(RC_series) {
		I = V_B/R_S;
		//use to find Z_box
		V_A = I(Z_box + R_s);
		//find phase angle
		//phase angle = (# divisions between leading and lagging wave)/(# divisions in one full waveform) * 360 degrees;
		//solve for R and C
		//solve for R then solve for C
		Z_box = mag(Z_box)*cos(phase angle) + mag(Z_box)*sin(phase angle) = R + 1/(2*pi*f*C);
		R = mag(Z_box)*cos(phase angle);
		1/(2*pi*f*c) = mag(Z_box) * sin(phase angle);
	}

	if(RC_parallel) {
		V_A = I((1/Y_box) + R_s);
		//find phase
		//phase angle = (# divisions between leading and lagging wave)/(# divisions in one full waveform) * 360 degrees;
		Z_box = mag(1/Y_box)*cos(phi) + j(1/Y_box)*sin(phi)
		//solve for C and R
		R = 1/((1/Y_box)*cos(phi))
		1/(2*pi*f*C) = 1/((1/Y_box)*sin(phi));
	}

	if(RL_series) {
		V_A = I(mag(Z_box) + R_S); 
		//phase angle = (# divisions between leading and lagging wave)/(# divisions in one full waveform) * 360 degrees;
		Z_box = mag(Z_box)*cos(phase angle) + j*mag(Z_box)*sin(phase angle);
		//solve for L and R
		mag(Z_box)*sin(phase angle) = 2*pi*f*L;
		R = mag(Z_box)*sin(phase angle);
	}

	if(RL_parallel) {
		V_A = I( 1/Y_box + R);
		Z_box = (1/Y_box)*cos(phase angle) + j*(1/Y_box)*sin(phase angle);
		//solve for L and R
		R = 1/(mag(1/Y_box)*cos(phase angle));
		2*pi*f*L = 1/(mag(1/Y_box)*sin(phase angle);
	}

}

```
