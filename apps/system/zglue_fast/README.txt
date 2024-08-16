README File for the FAST Management Tool
============================

The FAST Management tool provides a way to debug FAST related problems.  This README file
will provide usage information for the FAST tools.

CONTENTS
========

  o System Requirements
    - FAST Driver
    - Configuration Options
  o Help
  o Common Line Form
  o Common Command Options
    - "Sticky" Options
    - Environment variables
    - Common Option Summary
  o Command summary
    - bus
    - dev
    - get
    - set
    - verf
  o FAST Build Configuration
    - NuttX Configuration Requirements
    - FAST Tool Configuration Options

System Requirements
===================

The FAST tool is designed to be implemented as a NuttShell (NSH) add-on.  Read
the apps/nshlib/README.txt file for information about add-ons.

Configuration Options
---------------------
CONFIG_NSH_BUILTIN_APPS - Build the tools as an NSH built-in command

HELP
====

First of all, the I2C tools supports a pretty extensive help output.  That
help output can be view by entering either:

  nsh> fast help

or

  nsh> fast ?

Here is an example of the help output.  I shows the general form of the
command line, the various I2C commands supported with their unique command
line options, and a more detailed summary of the command I2C command
options.

  nsh> fast help
  Usage: fast <cmd> [arguments]
  Where <cmd> is one of:

    Show help     : ?
    List buses    : bus
    List devices  : dev [OPTIONS] <first> <last>
    Read register : get [OPTIONS] [<repititions>]
    Show help     : help
    Write register: set [OPTIONS] <value> [<repititions>]
    Verify access : verf [OPTIONS] <value> [<repititions>]

  Where common "sticky" OPTIONS include:
    [-a addr] is the I2C device address (hex).  Default: 03 Current: 03
    [-b bus] is the I2C bus number (decimal).  Default: 1 Current: 1
    [-r regaddr] is the I2C device register address (hex).  Default: 00 Current: 00
    [-w width] is the data width (8 or 16 decimal).  Default: 8 Current: 8
    [-s|n], send/don't send start between command and data.  Default: -n Current: -n
    [-i|j], Auto increment|don't increment regaddr on repititions.  Default: NO Current: NO
    [-f freq] I2C frequency.  Default: 100000 Current: 100000

  NOTES:
  o An environment variable like $PATH may be used for any argument.
  o Arguments are "sticky".  For example, once the I2C address is
    specified, that address will be re-used until it is changed.

  WARNING:
  o The I2C dev command may have bad side effects on your I2C devices.
    Use only at your own risk.

COMMAND LINE FORM
=================

The I2C is started from NSH by invoking the 'fast' command from the NSH
command line. The general form of the fast' command is:

  fast <cmd> [arguments]

Where <cmd> is a "sub-command" and identifies one FAST operations supported
by the tool.  [arguments] represents the list of arguments needed to perform
the FAST operation.  Those arguments vary from command to command as
described below.  However, there is also a core set of common OPTIONS
supported by all commands.  So perhaps a better representation of the
general FAST command would be:

  fast <cmd> [OPTIONS] [arguments]

Where [OPTIONS] represents the common options and and arguments represent
the operation-specific arguments.

COMMON COMMAND OPTIONS
======================

"Sticky" Options
----------------

In order to interact with FAST devices, there are a number of FAST parameters
that must be set correctly.  One way to do this would be to provide to set
the value of each separate command for each FAST parameter.  The FAST tool
takes a different approach, instead:  The FAST configuration can be specified
as a (potentially long) sequence of command line arguments.

These arguments, however, are "sticky."  They are sticky in the sense that
once you set the FAST parameter, that value will remain until it is reset
with a new value (or until you reset the board).

Environment Variables
---------------------
NOTE also that if environment variables are not disabled (by
CONFIG_DISABLE_ENVIRON=y), then these options may also be environment
variables.  Environment variables must be preceded with the special
character $.  For example, PWD is the variable that holds the current
working directory and so $PWD could be used as a command line argument.  The
use of environment variables on the FAST tools command is really only useful
if you wish to write NSH scripts to execute a longer, more complex series of
FAST commands.

Common Option Summary
---------------------

COMMAND SUMMARY
===============

FAST BUILD CONFIGURATION
=======================

NuttX Configuration Requirements
--------------------------------

FAST Management Tool Configuration Options
------------------------------

The default behavior of the FAST management tool can be modified by the setting the
options in the NuttX configuration.  This configuration is the defconfig
file in your configuration directory that is copied to the NuttX top-level
directory as .config when NuttX is configured.

  CONFIG_NSH_BUILTIN_APPS: Build the tools as an NSH built-in command
