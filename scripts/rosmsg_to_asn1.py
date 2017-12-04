#!/usr/bin/env python

from __future__ import print_function, division
import argparse
from enum import Enum
import getopt
import importlib
import logging
import os
import pprint
import rosmsg
import rospkg
import sys


ros_primitive_type = [
    'bool', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64',
    'uint64', 'float32', 'float64', 'string', 'time', 'duration']

cpp_type = [
    'uint8_t', 'int8_t', 'uint8_t', 'int16_t', 'uint16_t', 'int32_t',
    'uint32_t', 'int64_t', 'uint64_t', 'float', 'double', 'std::string',
    'ros::Time', 'ros::Duration']

python_type = [
    'bool', 'int', 'int', 'int', 'int', 'int', 'int' 'long', 'long', 'float',
    'float', 'str', 'rospy.Time', 'rospy.Duration']

ASN1_type = [
    'T-Boolean', 'T-Int8', 'T-UInt8', 'T-Int16', 'T-UInt16', 'T-Int32',
    'T-UInt32', 'T-Int64', 'T-UInt64', 'T-Float', 'T-Double', 'T-String',
    'Time', 'Duration']

primitive_types_map = {
    'bool': 'T-Boolean', 'int8': 'T-Int8', 'uint8': 'T-UInt8',
    'int16': 'T-Int16', 'uint16': 'T-UInt16', 'int32': 'T-Int32',
    'uint32': 'T-UInt32', 'int64': 'T-Int64', 'uint64': 'T-UInt64',
    'float32': 'T-Float', 'float64': 'T-Double', 'string': 'T-String',
    'time': 'Time', 'duration': 'Duration', 'byte': 'T-Int8',
    'char': 'T-Uint8'}

libraries = dict()

for key in ['T_Boolean', 'T-Int8', 'T-Uint8', 'T-Int32', 'T-UInt32']:
    libraries[key] = 'TASTE-BasicTypes'

for key in ['T-Int16', 'T-Uint16', 'T-Int64', 'T-Uint64', 'T-Float',
            'T-Double', 'T-String']:
    libraries[key] = 'TASTE-ExtendedTypes'

for key in ['Time', 'Duration']:
    libraries[key] = 'Time-Types'


class ErrorCodes (Enum):
    ARGS_ERROR = 1
    SYSCMD_ERROR = 2
    OK = 0


def parse_args():
    """
    Parse the command line arguments.
    :returns: Class containing the commandline options and arguments.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-o", "--output", default="/tmp/asn1_msgs/",
        help="Directory to save the ASN.1 messages. Default is /tmp/asn1_msgs")
    parser.add_argument(
        "-v", "--verbose", action='store_true',
        help="Verbose output of operation")
    parser.add_argument(
        "-l", "--log", action='store_true',
        help="Log the operation at the default path /tmp/rosmsg_to_asn1.log")
    parser.add_argument(
         "--logfile", nargs='?', help="Set path to log the operation.")
    parser.add_argument(
        "messages", nargs="*",
        help="The list of messages to be converted to ASN.1")

    return parser.parse_args()


def get_message_package(rospack, msg):
    """
    Get the package name a message belongs to. If the message is not found in
    any of the packages or in multiple packages a message is logged and a None
    value is returned.
    :param rospack: A RosPack instance.
    :param msg: The message to search the package for.
    :return msg_name: The name of the message (None if not found).
    :return pkg_name: The name of the package (None if not found).
    """
    msg_name = None
    pkg_name = None
    # If message is prefixed search in that package only.
    if "/" in msg:
        msgs = list(rosmsg.list_msgs(msg.split('/')[0], rospack))
        if msg not in msgs:
            logging.error("Couldn't find the message {0}".format(msg))
        else:
            pkg_name = msg.split('/')[0]
            msg_name = msg.split('/')[1]
    # If there is no prefix find all the messages with that name.
    # If multiple messages with the same name exist print a message and exit.
    else:
        messages = list(rosmsg.rosmsg_search(rospack, rosmsg.MODE_MSG, msg))
        if len(messages) > 1:
            s = ""
            comma = ""
            for m in messages:
                s += comma + m
                comma = ", "
            logging.error(
                "Found {0} messages with name {1}. Please clarify the one you"
                " want by prefixing the package name.\n"
                "[".format(len(messages), msg) + s + "]")
        elif len(messages) == 1:
            logging.debug("Found the message {0}".format(messages[0]))
            pkg_name = messages[0].split('/')[0]
            msg_name = messages[0].split('/')[1]
        else:
            logging.error("Couldn't find the message {0}".format(msg))

    return msg_name, pkg_name


def get_python_msg_module(pkg):
    """
    Returns the Python module that includes the of the given package.
    :param pkg: The name of the package.
    :return module: The Python module of the package (None if not found).
    """
    module_name = pkg + ".msg"
    module = None
    try:
        logging.debug("Trying to import {0}".format(module_name))
        module = importlib.import_module(module_name)
    except ImportError as e:
        logging.error(e.msg)
        logging.error(
            "Could not load the package {0}."
            "Is it in your package path?".format(module_name))
    return module


def get_message_dict(module, msg):
    """
    Returns the dictionary representing a message type from a module.
    :param module: The Python module contining the message.
    :param msg: The message type.
    """
    msg_dict = None
    obj = None
    try:
        # Create an object for introspection
        obj = module.__dict__[msg]
    except KeyError as e:
        logging.error(
            "Couldn't find message {0} in module {1}".format(msg, module))
        return msg_dict
    try:
        msg_dict = dict(
            type=obj._type, slots=obj.__slots__, slot_types=obj._slot_types,
            full_text=obj._full_text)
    except KeyError as e:
        logging.error(
            "Could not read the information of message {}.".format(msg))
        return None
    logging.debug("Dictionary for message {0} is:".format(msg))
    logging.debug(pprint.pformat(msg_dict))
    return msg_dict


def find_field_type(slot_type):
    """
    Finds the field type of a field. If it is an array it returns the data type
    of the array along the array type and its length.
    :param slot_type: The type of the slot whose field type is to be found.
    :return slot_type: The base data type of the slot (if it is an array).
    :return field_type: The field_type of the slot.
    :return length: The length of the array.
    """
    bracket = "["
    length = 1
    field_type = "simple"
    idx = slot_type.find(bracket)
    if idx == -1:
        return slot_type, field_type, length
    else:
        if slot_type[idx + 1] == "]":
            field_type = "variable-length"
        else:
            field_type = "fixed-length"
            idx_end = slot_type.find("]")
            str_length = slot_type[idx+1:idx_end]
            length = int(str_length)
        slot_type = slot_type[0:idx]
    return slot_type, field_type, length


def find_ASN1_type(slot_type):
    """
    Given the ROS type of a field finds and returns the equivalent ASN.1 type.
    If the type is not primitive a flag is set so that a dependency is
    generated.
    :param slot_type: The ROS field type.
    :return asn1_type: The ASN.1 equivalent type of the ROS type.
    :return lib_asn: The library the ASN.1 type belongs to.
    :return primitive: Flag to denote if the field type is primitive.
    """
    primitive = False
    if slot_type in ros_primitive_type:
        asn1_type = primitive_types_map[slot_type]
        lib_asn = libraries[asn1_type]
        primitive = True
    else:
        composed_type = slot_type.split("/")
        if len(composed_type) == 1:
            logging.warn("The length of the composed_type is 1!")
        asn1_type = composed_type[1]
        # primitive_types_map[slot_type] = asn1_type
        lib_asn = composed_type[1]+'-Types'
        # libraries[asn1_type] = lib_asn
        primitive = False
    return asn1_type, lib_asn, primitive


def generate_libraries_string(import_libraries):
    """
    Generate a string from a set of libraries to be imported to the ASN.1 type.
    :param import_libraries: The libraries to be imported.
    :return s: The libraries names in string format.
    """
    s = "IMPORTS "
    comma = ""
    for key in import_libraries:
        num_types = len(import_libraries[key])
        for t in import_libraries[key]:
            s += comma + t
            comma = ", "
        s += " FROM " + key + " "
        comma = ""
    s += ";"
    s += "\n"
    return s


def process_msg(rospack, msg):
    """
    Process a message creating a string containing an ASN.1 representation.
    :param rospack: A rospkg instance.
    :param msg: The ROS message to be converted to ASN.1 message.
    :return msg_string: The string containing the ASN.1 representation.
    :return msg_deps: Any message dependencies that need to be generated.
    """
    msg_name = None
    pkg_name = None
    msg_string = None
    msg_deps = None
    # Get the package contining the message
    msg_name, pkg_name = get_message_package(rospack, msg)
    if msg_name is None or pkg_name is None:
        return msg_string, msg_deps
    # Get the Python module contining the package's messages
    module = get_python_msg_module(pkg_name)
    if module is None:
        return msg_string, msg_deps

    # Get the message as a dictionary
    msg_dict = get_message_dict(module, msg_name)
    if msg_dict is None:
        return msg_string, msg_deps

    msg_string = (msg_name + "-Types DEFINITIONS ::=\nBEGIN\n")
    msg_deps = []
    slot_types = msg_dict["slot_types"]
    slots = msg_dict["slots"]
    num_fields = len(slot_types)

    import_libraries = dict()

    slot_asn1_types = []
    field_types = []
    lengths = []

    for slot_type in slot_types:
        slot_type, field_type, length = find_field_type(slot_type)
        asn1_type, lib_asn, primitive = find_ASN1_type(slot_type)
        if lib_asn in import_libraries:
            if asn1_type not in import_libraries[lib_asn]:
                import_libraries[lib_asn].append(asn1_type)
        else:
            import_libraries[lib_asn] = [asn1_type]
        slot_asn1_types.append(asn1_type)
        field_types.append(field_type)
        lengths.append(length)
        if not primitive:
            msg_deps.append(slot_type)

    msg_string += generate_libraries_string(import_libraries)
    fixed_indices = [i for i, x in enumerate(field_types)
                     if x == "fixed-length"]
    variable_indices = [i for i, x in enumerate(field_types)
                        if x == "variable-length"]

    for i in fixed_indices:
        msg_string += "L" + slots[i].replace("_", "-") + "::="
        msg_string += " SEQUENCE (SIZE(0.." + str(lengths[i])+")) OF "
        msg_string += slot_asn1_types[i] + "\n"
        field_types[i] = "simple"
        slot_asn1_types[i] = "L" + slots[i].replace("_", "-")

    for i in variable_indices:
        msg_string += "V" + slots[i].replace("_", "-") + "::="
        msg_string += " SEQUENCE (SIZE(0..256)) OF "
        msg_string += slot_asn1_types[i] + "\n"
        field_types[i] = "simple"
        slot_asn1_types[i] = "V" + slots[i].replace("_", "-")

    if (num_fields == 1):
        if field_types[0] == "simple":
            if slots[0] == "type":
                slots[0] += "-" + slot_asn1_types[0]
            if slots[0].lower() == slot_asn1_types[0].lower():
                slots[0] += "-field"
            msg_string += msg_name + "::=\nSEQUENCE\n{\n"
            msg_string += "\t" + slots[0].replace("_", "-") + "\t"
            msg_string += slot_asn1_types[0] + "\n}\n"
    else:
        msg_string += msg_name + "::=\nSEQUENCE\n{\n"
        comma = ""
        for i in range(num_fields):
            if field_types[i] == "simple":
                if slots[i] == "type":
                    slots[i] += "-" + slot_asn1_types[i]
                if slots[i].lower() == slot_asn1_types[i].lower():
                    slots[i] += "-field"
                msg_string += comma + "\t" + slots[i].replace("_", "-")
                msg_string += "\t" + slot_asn1_types[i]
                comma = ",\n"
        msg_string += "\n}\n"
    msg_string += "END"

    return msg_string, msg_deps


def save_msg(asn1_str, msg, out_dir):
    """
    Save the ASN.1 generated message to a file.
    :param asn1_str: The message string.
    :param msg: The message name.
    :param out_dir: The output directory.
    """
    if not os.path.exists(out_dir):
        os.makedirs(out_dir)
    if "/" in msg:
        msg = msg.split("/")[1]
    f = open(out_dir + msg + ".asn", "w")
    f.write(asn1_str)
    f.close()


def main():
    # Get the messages to be generated
    args = parse_args()
    print(args)
    messages = args.messages
    out_dir = args.output
    if out_dir[-1] != "/":
        out_dir += "/"

    log_formatter = logging.Formatter(
        "%(asctime)s [%(threadName)-12.12s] [%(levelname)-5.5s]  %(message)s")
    root_logger = logging.getLogger()
    if args.verbose:
        root_logger.setLevel(logging.DEBUG)

    if args.log:
        log_file = args.logfile
        if log_file is None:
            log_file = "/tmp/rosmsg_to_asn1.log"
        file_handler = logging.FileHandler(log_file)
        file_handler.setFormatter(log_formatter)
        if args.verbose:
            file_handler.setLevel(logging.DEBUG)
        root_logger.addHandler(file_handler)

    console_handler = logging.StreamHandler(sys.stdout)
    if args.verbose:
        console_handler.setLevel(logging.DEBUG)
    console_handler.setFormatter(log_formatter)
    root_logger.addHandler(console_handler)

    rospack = rospkg.RosPack()

    generated_msgs = []

    while len(messages) > 0:
        # Get message info
        msg = messages.pop(0)
        asn1_str, msg_deps = process_msg(rospack, msg)
        if asn1_str is None or msg_deps is None:
            logging.error("Error generating message {0}".format(msg))
            sys.exit(ErrorCodes.ARGS_ERROR.value)
        save_msg(asn1_str, msg, out_dir)
        generated_msgs.append(msg)
        for dep in msg_deps:
            # If the dep is not already generated and is not planned to be
            # generated
            if dep not in messages and dep not in generated_msgs:
                messages.append(dep)


if __name__ == "__main__":
    main()
