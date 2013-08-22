# # -*- coding: utf-8 -*- 

# Copyright 2013 Séverin Lemaignan
# http://www.ros.org/wiki/nao
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    # Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#    # Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#    # Neither the name of the University of Freiburg nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import os
import math
import xml.etree.ElementTree as ET

NS="{http://www.aldebaran-robotics.com/schema/choregraphe/position.xsd}"

def _makeJointDict(motors, use_radians = True):
    pose = {}
    for p in motors.findall(NS + "Motor"):
        name = p.find(NS + 'name').text
        value = float(p.find(NS + 'value').text)
        if not use_radians:
            value = math.radians(value)

        pose[name] = value

    return pose

def getpostures(xap_file):
    """ Parses a Aldebaran Choregraphe posture library (.xap files)
    into a Python dictionary of postures.
    """

    if not os.path.exists(xap_file):
        raise RuntimeError("The XAP file %s does not exist." % xap_file)

    try:
        tree = ET.parse(xap_file)
    except ET.ParseError:
        raise RuntimeError("The XAP file %s is not a valid XML file." % xap_file)


    root=tree.getroot()

    postures = {}


    positions = [p for p in root.iter(NS + 'position')]

    if not positions:
        raise RuntimeError("The XAP file %s does not contain any pose." % xap_file)

    for p in positions:
        name = p.find(NS + 'name').text
        version = p.find(NS + 'version') # it *seems* that the 'version' 2 indicates joints stored in radians
        pose = _makeJointDict(p.find(NS + "Motors"), version is not None and version.text=='2')

        postures[name] = pose

    return postures

if __name__ == "__main__":

    import sys
    if len(sys.argv) != 2:
        print("Usage: python xapparser.py <file.xap>")
        sys.exit(1)

    print(getpostures(sys.argv[1]))

