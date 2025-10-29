import React, { useState, useEffect } from 'react'
import './App.css'
import BatteryGauge from './components/BatteryGauge'
import BatteryChart from './components/BatteryChart'
import BatteryInfo from './components/BatteryInfo'

function App() {
  const [batteryData, setBatteryData] = useState({
    voltage: 0,
    percentage: 0,
    current: 0,
    temperature: 0,
    status: 'UNKNOWN',
    isCharging: false,
    lastUpdate: null
  })
  
  const [history, setHistory] = useState([])
  const [isConnected, setIsConnected] = useState(false)
  const [wsUrl, setWsUrl] = useState('ws://localhost:9090')

  useEffect(() => {
    // Connect to rosbridge websocket
    let ws = null
    let reconnectTimer = null

    const connect = () => {
      try {
        ws = new WebSocket(wsUrl)
        
        ws.onopen = () => {
          console.log('Connected to ROS bridge')
          setIsConnected(true)
          
          // Subscribe to battery_state topic
          const subscribeMsg = {
            op: 'subscribe',
            topic: '/battery_state',
            type: 'sensor_msgs/BatteryState'
          }
          ws.send(JSON.stringify(subscribeMsg))
        }
        
        ws.onmessage = (event) => {
          const data = JSON.parse(event.data)
          if (data.topic === '/battery_state' && data.msg) {
            const msg = data.msg
            const newData = {
              voltage: msg.voltage || 0,
              percentage: msg.percentage || 0,
              current: msg.current || 0,
              temperature: msg.temperature || 0,
              status: getStatusText(msg.power_supply_status),
              isCharging: msg.power_supply_status === 1,
              lastUpdate: new Date()
            }
            
            setBatteryData(newData)
            
            // Add to history (keep last 100 points)
            setHistory(prev => {
              const updated = [...prev, {
                time: new Date(),
                percentage: newData.percentage,
                voltage: newData.voltage
              }]
              return updated.slice(-100)
            })
          }
        }
        
        ws.onerror = (error) => {
          console.error('WebSocket error:', error)
          setIsConnected(false)
        }
        
        ws.onclose = () => {
          console.log('Disconnected from ROS bridge')
          setIsConnected(false)
          
          // Attempt to reconnect after 3 seconds
          reconnectTimer = setTimeout(() => {
            console.log('Attempting to reconnect...')
            connect()
          }, 3000)
        }
      } catch (error) {
        console.error('Connection error:', error)
        setIsConnected(false)
      }
    }

    connect()

    return () => {
      if (ws) {
        ws.close()
      }
      if (reconnectTimer) {
        clearTimeout(reconnectTimer)
      }
    }
  }, [wsUrl])

  const getStatusText = (status) => {
    const statusMap = {
      0: 'UNKNOWN',
      1: 'CHARGING',
      2: 'DISCHARGING',
      3: 'NOT_CHARGING',
      4: 'FULL'
    }
    return statusMap[status] || 'UNKNOWN'
  }

  return (
    <div className="app">
      <header className="app-header">
        <h1>🔋 TurtleBot3 Battery Monitor</h1>
        <div className={`connection-status ${isConnected ? 'connected' : 'disconnected'}`}>
          <span className="status-dot"></span>
          {isConnected ? 'Connected' : 'Disconnected'}
        </div>
      </header>

      <div className="dashboard">
        <div className="main-panel">
          <BatteryGauge 
            percentage={batteryData.percentage}
            voltage={batteryData.voltage}
            isCharging={batteryData.isCharging}
          />
        </div>

        <div className="side-panel">
          <BatteryInfo batteryData={batteryData} />
        </div>

        <div className="chart-panel">
          <BatteryChart history={history} />
        </div>
      </div>

      {!isConnected && (
        <div className="connection-warning">
          <p>⚠️ Not connected to ROS bridge</p>
          <p className="help-text">
            Make sure rosbridge is running: <code>ros2 launch rosbridge_server rosbridge_websocket_launch.xml</code>
          </p>
        </div>
      )}
    </div>
  )
}

export default App
