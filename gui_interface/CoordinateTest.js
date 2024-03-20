import React, { useState, useEffect } from 'react';
import { View, Text, StyleSheet } from 'react-native';
import Roslib from 'roslib';

const CoordinateTest = () => {
  const [points1, setPoints1] = useState([]);
  const [points2, setPoints2] = useState([]);
  const [points3, setPoints3] = useState([]);
  // const [points4, setPoints4] = useState([]);
  const [subscribed, setSubscribed] = useState(false);
  let pointCounter = 0;

  const renderGrid = () => {
    const grid = [];
    for (let row = 0; row < 80; row++) {
        const rowNodes = [];
        for (let col = 0; col < 50; col++) {
          rowNodes.push(<Node key={`node-${row}-${col}`} />);
        }
      grid.push(<View key={`row-${row}`} style={styles.row}>{rowNodes}</View>);
    }
    return grid;
  };
  useEffect(() => {
    if (subscribed) {
      const ros = new Roslib.Ros({
        url: 'ws://192.168.246.130:9090'
      });

    const topicListener1 = new Roslib.Topic({
      ros: ros,
      name: '/obstacle_csv_topic',
      messageType: 'interfaces/Obstacles'
    });

    const topicListener2 = new Roslib.Topic({
      ros: ros,
      name: '/checkpoints_csv_topic',
      messageType: 'interfaces/Checkpoints'
    });

    const topicListener3 = new Roslib.Topic({
      ros: ros,
      name: '/environment_csv_topic',
      messageType: 'interfaces/Environment'
    });

    // const topicListener4 = new Roslib.Topic({
    //   ros: ros,
    //   name: '/current_location_topic',
    //   messageType: 'interfaces/Current_Coords'
    // });

    const subscribeToRosTopic1 = (message) => {
      // Parse the message
      const newPoint = {
        key: pointCounter++,
        easting: message.easting,
        northing: message.northing,
        elevation: message.elevation,
        bounding_radius: message.bounding_radius,
      };
      setPoints1(prevPoints => [...prevPoints, newPoint]);
    };

    const subscribeToRosTopic2 = (message) => {
      const newPoint = {
        key: pointCounter++,
        easting: message.easting,
        northing: message.northing,
      };
      setPoints2(prevPoints => [...prevPoints, newPoint]);
    };

    const subscribeToRosTopic3 = (message) => {
      const newPoint = {
        key: pointCounter++,
        easting: message.easting,
        northing: message.northing,
      };
      setPoints3(prevPoints => [...prevPoints, newPoint]);
    };

    // const subscribeToRosTopic4 = (message) => {
    //   const newPoint = {
    //     key: pointCounter++,
    //     easting: message.easting,
    //     westing: message.westing,
    //     angle: message.angle,
    //   };
    //   setPoints4(prevPoints => [...prevPoints, newPoint]);
    // };

    topicListener1.subscribe(subscribeToRosTopic1);
    topicListener2.subscribe(subscribeToRosTopic2);
    topicListener3.subscribe(subscribeToRosTopic3);
    // topicListener4.subscribe(subscribeToRosTopic4);

    setSubscribed(true);
    return () => {
      topicListener1.unsubscribe();
      topicListener2.unsubscribe();
      topicListener3.unsubscribe();
      // topicListener4.unsubscribe();
      ros.close();
    };
   }
 }, [subscribed]);


  return (
      <View style={styles.gridContainer}>
      {renderGrid()}
      <TextComponent text="0" size={16} x={0} y={-0.45} color="black" />
      <TextComponent text="5" size={16} x={5} y={-2} color="black" />
      <TextComponent text="10" size={16} x={9.5} y={-2} color="black" />
      <TextComponent text="15" size={16} x={14.5} y={-2} color="black" />
      <TextComponent text="20" size={16} x={19.5} y={-2} color="black" />
      <TextComponent text="-5" size={16} x={-5.4} y={-2} color="black" />
      <TextComponent text="-10" size={16} x={-10.7} y={-2} color="black" />
      <TextComponent text="-15" size={16} x={-15.7} y={-2} color="black" />
      <TextComponent text="-20" size={16} x={-20.7} y={-2} color="black" />
      <TextComponent text="5" size={16} x={0} y={4.5} color="black" />
      <TextComponent text="10" size={16} x={-0.3} y={9.5} color="black" />
      <TextComponent text="15" size={16} x={-0.3} y={14.5} color="black" />
      <TextComponent text="20" size={16} x={-0.3} y={19.5} color="black" />
      <TextComponent text="25" size={16} x={-0.3} y={24.5} color="black" />
      <TextComponent text="30" size={16} x={-0.3} y={29.5} color="black" />
      <TextComponent text="35" size={16} x={-0.3} y={34.5} color="black" />
      <TextComponent text="-5" size={16} x={-0.3} y={-5.5} color="black" />
      <TextComponent text="-10" size={16} x={-0.7} y={-10.5} color="black" />
      <TextComponent text="-15" size={16} x={-0.7} y={-15.5} color="black" />
      <TextComponent text="-20" size={16} x={-0.7} y={-20.5} color="black" />
      <TextComponent text="-25" size={16} x={-0.7} y={-25.5} color="black" />
      <TextComponent text="-30" size={16} x={-0.7} y={-30.5} color="black" />
      <TextComponent text="-35" size={16} x={-0.7} y={-35.5} color="black" />
      <XAxis x={0} y={491.5} length={600} color="black" />
      <YAxis x={299} y={12} height={960} color="black" />
        {points1.map((point1, index) => (
          <Point key={`point1-${index}`} size={8 * point1.bounding_radius} x={point1.easting} y={point1.northing} color="red" />
        ))}
        {points2.map((point2, index) => (
          <Point key={`point2-${index}`} size={10} x={point2.easting} y={point2.northing} color="green" />
        ))}
        {points3.map((point3, index) => (
          <Point key={`point3-${index}`} size={8} x={point3.easting} y={point3.northing} color="purple" />
        ))}
        {/* {points4.map((point4, index) => (
          <Point key={`point4-${index}`} size={8} x={point4.easting} y={point4.westing} color="grey" />
        ))} */}
      </View>  
  );
};

const XAxis = ({ x, y, length, color }) => {
  const lineStyle = {
    position: 'absolute',
    left: x,
    top: y,
    width: length,
    height: 2, // Adjust the height of the line as needed
    backgroundColor: color,
  };

  return <View style={lineStyle} />;
};

const YAxis = ({ x, y, height, color }) => {
  const lineStyle = {
    position: 'absolute',
    left: x,
    top: y,
    width: 2, // Adjust the width of the line as needed
    height: height,
    backgroundColor: color,
  };

  return <View style={lineStyle} />;
};


const TextComponent = ({ text, size, x, y, color }) => {
  const textStyle = {
    position: 'absolute',
    left: ((x * 12) + 303.5) - size / 2,
    top: (483 - (y * 12)) - size / 2,
    fontSize: size,
    color: color,
  };
  return <Text style={textStyle}>{text}</Text>;
};


const Node = () => {
    return (
      <View style={styles.node} />
    );
  };

  

  const Point = ({ size, x, y, color }) => {
    const pointStyle = {
      position: 'absolute',
      left: ((x * 12) + 300) - size / 2,
      top: (492 - (y * 12)) - size / 2,
      width: size,
      height: size,
      backgroundColor: color, // You can customize the color here
      borderRadius: size / 2,
    };
  
  return <View style={pointStyle} />;
};


const styles = StyleSheet.create({
  gridContainer: {
    paddingTop: 12,
    flexDirection: 'column',
    justifyContent: 'center',
    alignItems: 'center',
  },
  row: {
    flexDirection: 'row',
  },
  node: {
    width: 10,
    height: 10,
    backgroundColor: 'white',
    margin: 1,
  },
});

export default CoordinateTest;