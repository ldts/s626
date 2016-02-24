# s626 Linux Xenomai driver
Sensoray 626 Multifunction Analog/Digital I/O

Driver for Sensoray 626 for Xenomai using Analogy.
It allows to use multiple I/O cards.

Please read before proceeding

# Building
Use command 
make all

# To load driver 
Use simple_load_driver.sh

Before loading driver for s626 card it first unloads standard s626 driver.
Also it loads xeno_analogy module.

# To unload driver 
Use simple_unload_driver.sh

It unloads s626 driver along with xeno_analogy.


# Additional information 

If the Sensoray 626 I/O board is not responding please consult for
more information. After loading driver it should state that
s626 driver initialized the card and it was attached.

Before loading driver consult _lspci_ if the Sensoray 626
I/O board is visible! Also if you are going to use more than one 
s626 card you have to consult _lspci_ for **slot** and **bus** number 
on which each card is present.
If you are going to use a single card then you can pass to s626 
API 0 bot for **bus** and **slot**. However it won't work 
if you have attached more than one I/O card to the PC.

# Examples
_app.c_ and _app_single.c_ provide comprehensive example how to use the driver 
in user space.




