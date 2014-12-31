import zmq
import zlib
import cPickle as pickle

class ZmqEmitter():
  '''Object for wrapping up all the zmq functionality in one place.'''

  def __init__(self, port='5556'):
    self.context = zmq.Context()
    self.socket = self.context.socket(zmq.PUB)
    self.socket.bind("tcp://*:%s" % port)
  
  def send_zipped_pickle(self, obj, topic='',flags=0, protocol=-1):
    """pack and compress an object with pickle and zlib."""
    pobj = pickle.dumps(obj, protocol)
    zobj = zlib.compress(pobj)
    print 'zipped pickle is %i bytes'%len(zobj)
    return self.socket.send_multipart((topic,zobj), flags=flags)


