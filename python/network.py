
import os

import logging

import pyric
import pyric.pyw as pyw

class Network(object):
    """Create and control monitor mode wifi interfaces"""

    def __init__(self):
        pass

    def monitor_interfaces(self):
        """Get a list of interfaces that support monitor mode"""

        # Get a list of all the interfaces.
        ret = []
        for iface in pyw.interfaces():
            # Check if this card support monitor mode
            try:
                card = pyw.getcard(iface)
                if 'monitor' in pyw.devmodes(card) and 'mon0' != card.dev:
                    # Add the card to the list
                    ret.append(iface)
            except pyric.error as e:
                pass

        # Return the collected list
        return ret

    def configure_interface(self, interface, name = 'mon0', channel = 1, txpower = 60, bitrate = 11):
        """Configure the given card in monitor mode"""

        # Get the card for this interface
        try:
            card = pyw.getcard(interface)
        except pyric.error as e:
            logging.error("Error connecting to the interface: " + interface)
            return None

        # Ensure this card supports monitor mode
        if 'monitor' not in pyw.devmodes(card):
            logging.error(interface + " does not support monitor mode")
            return None

        # Delete any previously created interfaces
        try:
            for iface in pyw.ifaces(card):
                if iface[0].dev == name:
                    logging.info("Deleting interface: " + iface[0].dev)
                    pyw.devdel(iface[0])
        except pyric.error as e:
            logging.error("Error trying to delete existing interfaces")
            logging.error(e)
            return None

        # Configure the bitrate for this card
        # This is not supported by pyric, so we have to do it manually.
        try:
            pyw.down(card)
            pyw.modeset(card, 'managed')
            pyw.up(card)
            logging.debug("Setting the bitrate on interface " + interface + " to " + str(bitrate))
            if os.system("iw dev " + card.dev + " set bitrates legacy-2.4 " + str(bitrate)) != 0:
            #if os.system("iwconfig " + card.dev + " rate 54M fixed") != 0:
                logging.error("Error setting the bitrate for: " + interface)
                return None
            pyw.down(card)
        except pyric.error as e:
            logging.error("Error setting the bitrate for: " + interface)
            logging.error(e)
            return None

        # Create a virtual monitor mode interface
        try:
            m0 = pyw.devadd(card, name, 'monitor')
        except pyric.error as e:
            logging.error("Error creating the monitor mode interface for: " + interface)
            logging.error(e)
            return None
        logging.info("Created network interface: " + name)

        # Try to configure the transmit power level (some cards don't support this)
        try:
            pyw.txset(m0, txpower, 'fixed')
        except pyric.error as e:
            pass

        # Bring the interface up
        try:
            pyw.up(m0)
        except pyric.error as e:
            logging.error("Error bringing up the interface: " + m0.dev)
            logging.error(e)
            return False

        # Configure the channel
        try:
            logging.debug("Changing to channel: " + str(channel))
            pyw.chset(m0, channel, None)
        except pyric.error as e:
            logging.error("Error setting the wifi channel on: " + m0.dev)
            logging.error(e)
            return False

        return m0
