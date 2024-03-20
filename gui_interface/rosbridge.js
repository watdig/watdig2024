import Roslib from 'roslib';

const setupRosBridge = () => {
  const ros = new Roslib.Ros({
    url: 'ws://YOUR_ROSBRIDGE_SERVER_IP:9090'
  });

  const topicListener = new Roslib.Topic({
    ros: ros,
    name: '/obstacles',
    messageType: 'interfaces/obstacles'
  });

  const subscribeToRosTopic = (callback) => {
    topicListener.subscribe(message => {
      // Parse the message
      const data = {
        name: message.name,
        easting: message.easting,
        northing: message.northing,
        elevation: message.elevation,
        bounding_radius: message.bounding_radius
      };
      callback(data);
    });
  };

  const closeRosConnection = () => {
    topicListener.unsubscribe();
    ros.close();
  };

  return { subscribeToRosTopic, closeRosConnection };
};

export default setupRosBridge;
