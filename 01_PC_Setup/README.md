# Setting up your Computer

This course requires software that only runs on Linux.  To help you out, a Linux "virtual machine" has been created for you.

You will need 25GB of free space on your computer.  **Do not procede any further until you have freed up some space on your hard drive.**


## Install the Virtual Machine

These steps were adapted from https://www.pyimagesearch.com/2017/09/22/deep-learning-python-ubuntu-virtual-machine/.  (This is a great resource for all kinds of cool projects...I highly recommend that you check out the other tutorials on this Website.)

1. Download VirtualBox:  https://www.virtualbox.org/wiki/Downloads 
	- The "host" is your computer.  For example, if you're running Windows, choose a Windows host.

	- Also go ahead and download the VirtualBox Extension Pack.  The same file works for all supported platforms (e.g., Windows or Mac)
	
	- You do NOT need to download the SDK.

2. Install VirtualBox.

3. Install the Guest Additions by double-clicking on the extension pack file that you downloaded above.

4. Download the virtual machine for the course.
	- **The download link is provided to you on UBlearns**
	- Note:  This is a large file (4.4 GB)...it will take some time to download. 

5. The file you just downloaded has been "zipped" (compressed).  You'll need to unzip it now.

6.  Import the virtual machine:

	- Open VirtualBox
	- Select `File => Import Appliance`
	- Navigate to where the `ubuntu_1404_vm_F18.ova` file is saved on your computer.
	- Check the box for "Reinitialize the MAC address of all network cards".
	- Click "import"
			
7.  Start your virtual machine:

	- From the VirtualBox Manager, select the Ubuntu VM on the left.  Then click "Start"
	
	- When the virtual machine starts, you'll be prompted for a username and password.
		- Username:  student
		- Password:  learn
	
8.  Double-check that the Guest Additions have been installed.
	
	- From the VirtualBox top menu, select `Devices => Install Guest Additions...`.

9.  Shut down properly.
	
	- Don't simply exit from VirtualBox. 
	- Instead, it's best to shutdown Ubuntu first.	

---

## Troubleshooting

- If you get an error message like "VT-x/AMD-V hardware acceleration is not available for your system":
	- Disable Hyper-V mode from the Windows control panel (if you're using Windows).  See this link for more info:  https://superuser.com/questions/597121/vt-x-amd-v-hardware-acceleration-is-not-available-on-your-system 
	
	- Check your BIOS.  You'll need to re-boot your computer.  When it starts to boot, you'll need to use one of the Function keys (e.g., F2, F4, or F12) to enter "Setup".  From there, make sure that Virtualization is **enabled**.  This might be under an "advanced setting".  	
	

--- 

		
	
		
		

		
		

