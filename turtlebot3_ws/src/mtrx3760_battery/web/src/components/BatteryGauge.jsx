import React from 'react'
import './BatteryGauge.css'

function BatteryGauge({ percentage, voltage, isCharging }) {
  const getColor = () => {
    if (percentage >= 60) return '#28a745'
    if (percentage >= 30) return '#ffc107'
    return '#dc3545'
  }

  const getStatus = () => {
    if (isCharging) return 'CHARGING'
    if (percentage >= 60) return 'HEALTHY'
    if (percentage >= 30) return 'LOW'
    return 'CRITICAL'
  }

  return (
    <div className="battery-gauge">
      <div className="gauge-container">
        <svg viewBox="0 0 200 200" className="gauge-svg">
          {/* Background circle */}
          <circle
            cx="100"
            cy="100"
            r="80"
            fill="none"
            stroke="#e0e0e0"
            strokeWidth="20"
          />
          
          {/* Progress circle */}
          <circle
            cx="100"
            cy="100"
            r="80"
            fill="none"
            stroke={getColor()}
            strokeWidth="20"
            strokeDasharray={`${(percentage / 100) * 502.4} 502.4`}
            strokeLinecap="round"
            transform="rotate(-90 100 100)"
            className="gauge-progress"
          />
          
          {/* Center text */}
          <text
            x="100"
            y="95"
            textAnchor="middle"
            className="gauge-percentage"
            fill={getColor()}
          >
            {Math.round(percentage)}%
          </text>
          
          <text
            x="100"
            y="115"
            textAnchor="middle"
            className="gauge-voltage"
            fill="#666"
          >
            {voltage.toFixed(2)}V
          </text>
        </svg>
        
        {isCharging && (
          <div className="charging-indicator">⚡</div>
        )}
      </div>
      
      <div className="gauge-status" style={{ color: getColor() }}>
        {getStatus()}
      </div>
    </div>
  )
}

export default BatteryGauge
