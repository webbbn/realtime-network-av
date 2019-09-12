
import os

import logging
import subprocess

import pyric
import pyric.pyw as pyw
import pyric.utils.hardware as pywhw
import enum

class Card(enum.Enum):
    ath9k = 1
    rtl88xx = 2

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

        # Determine the type of card for this interface
        try:
            driver = pywhw.ifcard(interface)[0]
            print(driver)
            if driver == 'rtl88xxau':
                type = Card.rtl88xx
            else:
                type = Card.ath9k
        except Exception as e:
            print(e)
            return None

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

        # Configure the bitrate for this card
        # This is not supported by pyric, so we have to do it manually.
        if bitrate != 0 and type == Card.ath9k:
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

        # Try to configure the transmit power level (some cards don't support this)
        try:
            pyw.txset(card, txpower, 'fixed')
        except pyric.error as e:
            pass

        # Bring the interface up
        try:
            pyw.up(card)
        except pyric.error as e:
            logging.error("Error bringing up the interface: " + card.dev)
            logging.error(e)
            return False

        # Configure the channel
        try:
            logging.debug("Changing to channel: " + str(channel))
            pyw.chset(card, channel, None)

        except pyric.error as e:
            logging.error("Error setting the wifi channel on: " + card.dev)
            logging.error(e)
            return False

        return card
