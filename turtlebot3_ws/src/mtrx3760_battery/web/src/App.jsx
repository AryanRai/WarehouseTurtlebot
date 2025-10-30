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
  const [ws, setWs] = useState(null)

  useEffect(() => {
    // Connect to rosbridge websocket
    let wsConnection = null
    let reconnectTimer = null

    const connect = () => {
      try {
        wsConnection = new WebSocket(wsUrl)
        
        wsConnection.onopen = () => {
          console.log('Connected to ROS bridge')
          setIsConnected(true)
          setWs(wsConnection)
          
          // Subscribe to battery_state topic
          const subscribeMsg = {
            op: 'subscribe',
            topic: '/battery_state',
            type: 'sensor_msgs/BatteryState'
          }
          wsConnection.send(JSON.stringify(subscribeMsg))
        }
        
        wsConnection.onmessage = (event) => {
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
        
        wsConnection.onerror = (error) => {
          console.error('WebSocket error:', error)
          setIsConnected(false)
          setWs(null)
        }
        
        wsConnection.onclose = () => {
          console.log('Disconnected from ROS bridge')
          setIsConnected(false)
          setWs(null)
          
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
      if (wsConnection) {
        wsConnection.close()
      }
      if (reconnectTimer) {
        clearTimeout(reconnectTimer)
      }
    }
  }, [wsUrl])

  const handleReturnHome = () => {
    if (!ws || !isConnected) {
      alert('Not connected to ROS bridge')
      return
    }

    // Call the return home service
    const serviceCall = {
      op: 'call_service',
      service: '/return_home',
      type: 'std_srvs/Trigger'
    }
    
    ws.send(JSON.stringify(serviceCall))
    console.log('Return home service called')
    alert('Return home command sent!')
  }

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
          
          <div className="control-panel">
            <h3>Robot Control</h3>
            <button 
              className="return-home-btn"
              onClick={handleReturnHome}
              disabled={!isConnected}
            >
              🏠 Return to Home
            </button>
            {batteryData.percentage > 0 && batteryData.percentage < 20 && (
              <div className="low-battery-warning">
                ⚠️ Low Battery ({batteryData.percentage.toFixed(1)}%) - Auto return home activated
              </div>
            )}
          </div>
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
