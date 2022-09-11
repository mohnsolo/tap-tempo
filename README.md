Hardware:

TODO:
- Fill out Hardware section
- Add schematic
DONE - Eliminate magic numbers
DONE - Add a tuning routine that occurs on startup. This could be done once using a special input sequence, such as holding div button while pressing tap. The parameters would need to be saved to eeprom. Or, a simpler way is to always run the routine at startup, trashing the parameters every time the pedal is powered off. The routine will need the clock output from the PT2399 (at full speed or divided?). 
- Investigate why the latest version of the Bounce2 library did not compile.
- Consider making the tempo LED an output rather than the large clock divider circuit. One benefit could be to display the tapped-in tempo rather the true delay tempo. This is useful if divisions other than 1 are selected.