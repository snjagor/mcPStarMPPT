mcPStarMPPT 
===========

### an opensource app for controlling ProStarMPPT* solar charge controllers

## Requirements

* [libmodbus](https://libmodbus.org) C library is needed to build from source. 

* A C compiler is needed to build from source. On Windows and OSX you might have to install
developer tools. On Linux OSes `cc` should already be installed.

* Bash and perl are needed for testing script. Unless you are testing on windows this wont be a problem.

* This program only works for [Morningstar ProStarMPPT](https://www.morningstarcorp.com/) 
solar charge controllers. For Morningstar TriStarMPPT* charge controllers, see related 
[distribution](https://github.com/snjagor/mcTStarMPPT).

* You will need an appropriate device for connecting to the modbus terminal on the 
solar charger. (ie. Morningstar USB MeterBus Adapter)


## Installing

Once built, connect to your solar charger using the appropriate connector, open a shell terminal,
and run the program.

Once built you can use the program's `convert` command utility to create charge profiles if needed.

### Building from source

There are numerous user configuration settings at the top of the source code, (mostly dealing with
output file locations & names, and the [optional] date cache) a few of which are required & specific 
to your battery system. In the `main` function built-in defaults can be added to the data structure, 
otherwise in normal usage **_Profiles_** can be created to update the charger with various settings. 
Built-in defaults are used to populate newly created profiles when not connected to charger, and to
update the charger when the optional `revert` cmd is enabled.
(Some of these will probably be externalized to a config file in the near future.)

First install the *libmodbus* package. then..

Compile the source using cc:

```
cc -Wall $(pkg-config libmodbus --cflags) mcStarMPPT.c $(pkg-config libmodbus --libs) -lm -o mcPStarMPPT
```

### Multiple Chargers / Multiple Battery systems

[for now] multiple chargers on a network will each need a build of their own, configuring different modbus 
ids for each build, and renaming additional builds (using the above `-o` option) so as to avoid conflicts.
Its always a good idea to name builds to reflect what batteries the source was built for. 
ie. `mcPStarMPPT_L16-array`, etc...


## Documentation

Before changing any settings on your charger its a good idea to make backup settings files of 
the various charger default modes. Use the arguments 'profile backup' to do so for each of the seven
programmed dip switch modes, and rename the backup files to correspond to the documented modes so you 
can use them in the future. (Charger mode [8] for custom settings overwrites the charger's built in 
defaults it seems.) (This is not required, only a suggestion. Atleast one backup should be made before 
changing dip switches to custom mode so that common eeprom settings can be recorded.)

*Please read all safety and documentation details for your battery system, or you risk damage to 
yourself and/or the batteries.* This program has no way of knowing what values are correct beyond 
the generic defaults of the charger, (and those are often not the most appropriate) your batteries 
may become warranty void if using different charge values, so its a good idea to recheck what the 
required values are. 

Use the program argument `-h` to see a quick help guide.

With no arguments the program will output current stats and settings for the connected charger. 

### Command arguments:

- `poll` : shorter stats display
- `eeprom` : just output the configurable charger settings
- `json` : output data into a json file for external use

- `logs` : output logs from either days-ago, hourmeter, or memory address

	`logs n [options] [start x || days x || hrs x]`
	 -   `n` being: number-to-return, `x` = [start addr || days ago || hrs hourmeter]
	- Options: 
	  -	`-b n | --buffer n`  :buffer log dates with n hours
	  -	`--debug`  :additional log debug output
	- Output options: can be specified before `logs` cmd..
	- `logs all` will read All logs from charger, upto max.
	 
- `profile` : input updates and output backups

	`profile [nameof | validate nameof | create | backup]`
	 - `profile create` - will output built-in defaults if offline, otherwise live values

- `update` : update using default profile if one is set

- `revert` : update charger with program's built-in values (disabled by default)

- `convert` : convert between Float16 and Float values

**Coil commands:**
* `reboot` : reboot charger
* `EQ` : toggle a manual EQ cycle
* `reset_charge` : reset resetable daily charge totals
* `reset_totals` : reset all charge totals
* `reset_all` : clear all resetables 

**Output options:**
* default is converted decimal values
* `raw` : output raw values from memory (Float16, etc..) instead of human readable decimal
* `debug` : output additional debug data
* `-d` : dont print stats display 


### Read Examples
```
mcPStarMPPT
mcPStarMPPT json

mcPStarMPPT poll
mcPStarMPPT json poll

mcPStarMPPT debug -d	- just display ram and eeprom data
mcPStarMPPT eeprom
mcPStarMPPT raw eeprom

mcPStarMPPT logs 3 days 2
mcPStarMPPT json logs 2 days 1
```

### Modifying / Profile Examples:
```
mcPStarMPPT reboot
mcPStarMPPT reset_charge
mcPStarMPPT EQ

mcPStarMPPT profile backup

mcPStarMPPT profile create

mcPStarMPPT profile validate anewprofile	- validate file anewprofile_profile.txt

mcPStarMPPT profile idle	- update charge controller with 'idle' profile values

mcPStarMPPT update 		- (if a default profile is enabled.)
```

## Known Bugs

'days since EQ' eeprom register read not working..

### LogCache

For exactness, manually added logcache dates should be in sync with their related hourmeters so that they reflect similar times. Otherwise unexpected time gaps can be introduced when dating logs. If the manually added date is for a particular log, then use the log's hourmeter and create a timestamp for the evening of the desired day. Another way is to take a previous or later logcache time and add/subtract the same amount of time to/from the hourmeter and timestamp to get new ones. (ie. -40hrs to later hourmeter & -144000 seconds to later timestamp) Remember all manually added lines MUST be inserted in numerical hourmeter order to be recognized.

When the solar charger is turned on after a period of being off the logcache may tag the charger's hourmeter to the wrong date, which will confuse getting the correct date when reading logs. The easiest fix is to manually change the timestamp date in the logcache file for that hourmeter to last day of previous charger use, or if unknown, delete the the wrong logcache line (last one) after about 12 hrs of being back on (this will enable a new correct logcache date to be written).

## License

Released under the Artistic-2.0 license. Please read the LICENSE file for details.
Most use cases are allowed..

ProStarMPPT & TriStarMPPT are registered trademarks of Morningstar Corp.
Please see their [website](https://www.morningstarcorp.com/) for more info on their solar chargers.

## Contributing

See git repos on [source code][github.com] or [codeberg][codeberg.org] bug forums.. or clone and contribute.
If this program has been helpful please consider donating to the author at: [official website][]


## To Do

Planned features for working on include:
- [ ] better compatibility with multi-charger setups
- [ ] easier ability to change language of display output
- [ ] external settings configuration
- [ ] optional [-s] silent mode for library-like returns

[source code]: https://github.com/snjagor/mcPStarMPPT
[codeberg]: https://codeberg.org/snjagor/mcPStarMPPT
[official website]: https://www.centerflowing.com/#programs
