# Copyright 2009-2011 Armin Hornung, University of Freiburg
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

from threading import Thread

import rospy

# import Aldebaran API (must be in PYTHONPATH):
try:
    from naoqi import ALProxy
except ImportError:
    raise RuntimeError("Error importing NaoQI. Please make sure that Aldebaran's NaoQI API is in your PYTHONPATH.")

class NaoNode(Thread):
    """
    A ROS Node wrapper that can help you connect to NAOqi and deal with ROS shutdown
    To start your node, just call:
    my_node = MyNode('my_node')
    my_node.start() # that will spawn your node in a thread (and run whatever is in the run() function
    rospy.spin()
    # when killing ROS, the node will automatically stop its main loop, exit, and then unsubscribe from ALMemory events
    # and call whatever you have in unsubscribe()
    Then, if your node needs to process data, you just needs to have a run function:

    def run(Self):
        #do some initialization
        while self.is_looping():
            # do something
        # do some post processing
    """
    def __init__(self, name):
        """
        :param name: the name of the ROS node
        """
        super(NaoNode, self).__init__()

        # A distutils.version.LooseVersion that contains the current verssion of NAOqi we're connected to
        self.__naoqi_version = None
        self.__name = name

        ## NAOqi stuff
        # dict from a modulename to a proxy
        self.__proxies = {}

        # If user has set parameters for ip and port use them as default
        default_ip = rospy.get_param("~pip", "127.0.0.1")
        default_port = rospy.get_param("~pport", 9559)

        # get connection from command line:
        from argparse import ArgumentParser
        parser = ArgumentParser()
        parser.add_argument("--pip", dest="pip", default=default_ip,
                          help="IP/hostname of parent broker. Default is 127.0.0.1.", metavar="IP")
        parser.add_argument("--pport", dest="pport", default=default_port, type=int,
                          help="port of parent broker. Default is 9559.", metavar="PORT")

        import sys
        args = parser.parse_args(args=rospy.myargv(argv=sys.argv)[1:])
        self.pip = args.pip
        self.pport = args.pport

        ## ROS stuff
        self.__stop_thread = False
        rospy.init_node(self.__name)
        # make sure that we unregister from everything when the module dies
        rospy.on_shutdown(self.__on_ros_shutdown)

    def __on_ros_shutdown(self):
        """
        Callback function called whenever rospy.spin() stops
        """
        rospy.loginfo('Stopping ' + self.__name)

        self.__stop_thread = True
        # wait for the thread to be done
        if self.is_alive():
            self.join()

        rospy.loginfo(self.__name + ' stopped')

    def run(self):
        """
        This is a virtual method that corresponds to the code of the Node that runs continuously
        It should have a while loop calling the self.is_looping() function
        """
        """
        # code example
        #do some initialization
        while self.is_looping():
            # do something
        # do some post processing
        """
        raise NotImplementedError('Implement the run function of your NaoNode !')

    def is_looping(self):
        """
        :return: whether the thread is supposed to be running
        """
        return not self.__stop_thread

    def get_proxy(self, name, warn=True):
        """
        Returns a proxy to a specific module. If it has not been created yet, it is created
        :param name: the name of the module to create a proxy for
        :return: a proxy to the corresponding module
        """
        if name in self.__proxies and self.__proxies[name] is not None:
            return self.__proxies[name]

        proxy = None
        try:
            proxy = ALProxy(name,self.pip,self.pport)
        except RuntimeError,e:
            if warn:
                rospy.logerr("Could not create Proxy to \"%s\". \nException message:\n%s",name, e)

        self.__proxies[name] = proxy
        return proxy

    def get_version(self):
        """
        Returns the NAOqi version.
        A proxy for ALMemory is automatically created if needed as self.memProxy.
        You can then simply have code that runs or not depending on the NAOqi version.
        E.g. if distutils.version.LooseVersion('1.6') < get_version()    ....
        :return: a distutils.version.LooseVersion object with the NAOqi version
        """
        if self.__naoqi_version is None:
            proxy = self.get_proxy('ALMemory')
            if proxy is None:
                # exiting is bad but it should not happen
                # except maybe with NAOqi versions < 1.6 or future ones
                # in which case we will adapt that code to call the proper
                # version function
                exit(1)

            from distutils.version import LooseVersion
            self.__naoqi_version = LooseVersion(proxy.version())

        return self.__naoqi_version
