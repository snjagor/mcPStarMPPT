mcPStarMPPT 
===========

<a href="path-to-websites"></a>
This is a...



## Requirements

* [libmodbus]() C library is needed to build from source. 

* A C compiler is needed to build from source. On windows and OSX you might have to install
developer tools. On Linux OSes <cc> should already be installed.

* This program only works for [Morningstar ProStarMPPT]() solar charge controllers. For 
[Morningstar TriStarMPPT]() charge controllers see related distribution <...> .

* You will need an appropriate device for connecting to the modbus terminal on the solar charger.


## Installing

Once built, connect to your solar charger using the appropriate connector, open a shell terminal,
and run the program.

Once built you can use the program's 'convert' command utility to create charge profiles if needed.

### Building from source

There are numerous user configuration settings at the top of the source code, (mostly dealing with
output file locations & names, and the [optional] date cache) a few of which are required & specific 
to your battery system. In the main function built-in defaults can be added to the data structure, 
otherwise in normal usage Profiles can be created to update the charger with various settings. 
Built-in defaults are used to populate newly created profiles when not connected to charger, and to
update the charger when the optional 'update' cmd is enabled.
(Some of these will probably be externalized to a config file in the near future.)

First install the libmodbus package. then..

Compile the source using cc:

cc -Wall $(pkg-config libmodbus --cflags) mcPStarMPPT.c $(pkg-config libmodbus --libs) -lm -o mcPStarMPPT

### Multiple Chargers / Multiple Battery systems

[for now] multiple chargers on a network will each need a build of their own, configuring different modbus 
ids for each build, and renaming additional builds (using the above '-o' option) so as to avoid conflicts.
Its a good idea to rename builds to reflect what batteries the source was built for. 
ie. 'mcPStarMPPT_L16_array', etc...


## Documentation

Before changing any settings on your charger its a good idea to make backup settings files of 
the various charger default modes. Use the arguments 'profile backup' to do so for each of the six
toggle switch modes, and rename the backup files to correspond to the documented modes so you can 
use them in the future. Charger mode [7] for custom settings overwrites the charger's built in 
defaults it seems.

Please read all safety and documentation details for your battery system, or you risk damage to 
yourself and/or the batteries. This program has no way of knowing what values are correct beyond 
the generic defaults of the charger, (and those are often not the most appropriate) your batteries 
may become warranty void if using different charge values, so its a good idea to recheck what the 
required values are. 

use the program argument '-h' to see a quick help guide.

With no arguments the program will output current stats and settings for the connected charger. 

###Command arguments:

- 'poll' : shorter stats display
- 'eeprom' : just output the configurable charger settings
- 'json' : for most cmds will output data into a json file

- Coil commands:
* 'reboot' : reboot charger
* 'EQ' : toggle a manual EQ cycle
* 'reset_charge' : reset resetable daily charge totals
* 'reset_totals' : reset all charge totals
* 'reset_all' : clear all resetables 

- 'logs' : output logs from either days-ago, hourmeter, or memory address

	logs n [options] [start x || days x || hrs x]
	 -   n being: number-to-return, x = [start addr || days ago || hrs hourmeter]
	 Options: 
		-b n | --buffer n  :buffer log dates with n hours
		--debug  :additional log debug output
	 Output options: can be specified before logs cmd..
	 'logs all' will read All logs from charger, upto max.
	 
- 'profile' : input updates and output backups
	profile [nameof | validate nameof | create | backup]
	'profile create' - will output built-in defaults if offline, otherwise live values

- 'update' : update using default profile if one is set

- 'revert' : update charger with program's built-in values (disabled by default)

- 'convert' : convert between Float16 and Float values

- Output options:
* default is converted decimal values
* 'raw' : output raw values from memory instead of human readable decimal (Float16, etc..)


### Read Examples

mcPStarMPPT
mcPStarMPPT json

mcPStarMPPT poll
mcPStarMPPT json poll

mcPStarMPPT eeprom

mcPStarMPPT raw eeprom

mcPStarMPPT logs 2 days 2
mcPStarMPPT json logs 2 days 1

### Modifying / Profile Examples:

mcPStarMPPT reboot
mcPStarMPPT reset_charge
mcPStarMPPT EQ

mcPStarMPPT profile backup

mcPStarMPPT profile create

mcPStarMPPT profile validate anewprofile

mcPStarMPPT profile idle

mcPStarMPPT update - if enabled.


## License

Released under the Artistic-2.0 license. Please read the LICENSE file for details.
Most use cases are allowed..


## Contributing

See git repo bug forum.. or clone and create a patch?..


## To Do

* Planned features for working on include:
- better compatibility with multi-charger setups
- easier ability to change language of display output
- external settings configuration
- optional [-s] silent mode for library-like returns

