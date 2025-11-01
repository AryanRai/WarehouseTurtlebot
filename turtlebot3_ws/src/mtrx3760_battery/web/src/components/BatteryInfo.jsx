import React from 'react'
import './BatteryInfo.css'

function BatteryInfo({ batteryData }) {
  const formatTime = (date) => {
    if (!date) return 'N/A'
    return date.toLocaleTimeString()
  }

  const estimateTimeRemaining = () => {
    if (batteryData.isCharging) return 'Charging...'
    if (batteryData.percentage <= 0) return '0 min'
    
    // Rough estimate: assume 1% = 2 minutes
    const minutes = Math.round(batteryData.percentage * 2)
    const hours = Math.floor(minutes / 60)
    const mins = minutes % 60
    
    if (hours > 0) {
      return `~${hours}h ${mins}m`
    }
    return `~${mins}m`
  }

  return (
    <div className="battery-info">
      <h2>Battery Details</h2>
      
      <div className="info-grid">
        <div className="info-item">
          <span className="info-label">Voltage</span>
          <span className="info-value">{batteryData.voltage.toFixed(2)} V</span>
        </div>
        
        <div className="info-item">
          <span className="info-label">Percentage</span>
          <span className="info-value">{batteryData.percentage.toFixed(2)} %</span>
        </div>
        
        <div className="info-item">
          <span className="info-label">Temperature</span>
          <span className="info-value">{batteryData.temperature.toFixed(1)} °C</span>
        </div>
        
        <div className="info-item">
          <span className="info-label">Charge</span>
          <span className="info-value">{(batteryData.voltage * batteryData.current).toFixed(2)} Ah</span>
        </div>
        
        <div className="info-item full-width">
          <span className="info-label">Last Update</span>
          <span className="info-value">{formatTime(batteryData.lastUpdate)}</span>
        </div>
      </div>
    </div>
  )
}

export default BatteryInfo
