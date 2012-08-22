/**
 * struct tsc_axis	Touchscreen axis configuration
 * @min:		Minimum touch boundary for the axis
 * @max:		Maximum touch boundary for the axis
 * @inverted:		Flag to indicate the axis is inverted
 */

struct tsc_axis {
	u16 min;
	u16 max;
	u16 inverted;
};

/**
 * struct tsc_data	Touchscreen wire configuration
 * @wires:		Wires refer to application modes
 *			i.e. 4/5/8 wire touchscreen support
 *			on the platform
 * @x:			Minimum and Maximum boundary for X axis.
 * @y:			Minimum and Maximum boundary for Y axis.
 * @x_plate_resistance:	X plate resistance.
 * @steps_to_configure: The sequencer supports a total of
 *			16 programmable steps.
 *			A step configured to read a single
 *			co-ordinate value, can be applied
 *			more number of times for better results.
 */

struct tsc_data {
	int wires;
	struct tsc_axis x;
	struct tsc_axis y;
	int x_plate_resistance;
	int steps_to_configure;
};
