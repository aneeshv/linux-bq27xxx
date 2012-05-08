/**
 * struct tsc_data	Touchscreen wire configuration
 * @wires:		Wires refer to application modes
 *			i.e. 4/5/8 wire touchscreen support
 *			on the platform
 * @x_plate_resistance:	X plate resistance.
 */

struct tsc_data {
	int wires;
	u16 x_min, x_max;
	u16 y_min, y_max;
	int x_plate_resistance;
};
