import React from 'react'
import './BatteryChart.css'

function BatteryChart({ history }) {
  if (history.length === 0) {
    return (
      <div className="battery-chart">
        <h2>Battery History</h2>
        <div className="no-data">No data available yet...</div>
      </div>
    )
  }

  const maxPercentage = 100
  const minPercentage = 0
  const width = 800
  const height = 200
  const padding = 40

  // Calculate points for the line
  const points = history.map((point, index) => {
    const x = padding + (index / (history.length - 1)) * (width - 2 * padding)
    const y = height - padding - ((point.percentage - minPercentage) / (maxPercentage - minPercentage)) * (height - 2 * padding)
    return `${x},${y}`
  }).join(' ')

  // Get time labels
  const getTimeLabel = (date) => {
    return date.toLocaleTimeString('en-US', { hour: '2-digit', minute: '2-digit' })
  }

  const firstTime = history[0]?.time
  const lastTime = history[history.length - 1]?.time
  const currentPercentage = history[history.length - 1]?.percentage || 0

  return (
    <div className="battery-chart">
      <h2>Battery History</h2>
      
      <div className="chart-container">
        <svg viewBox={`0 0 ${width} ${height}`} className="chart-svg">
          {/* Grid lines */}
          {[0, 25, 50, 75, 100].map(percent => {
            const y = height - padding - ((percent - minPercentage) / (maxPercentage - minPercentage)) * (height - 2 * padding)
            return (
              <g key={percent}>
                <line
                  x1={padding}
                  y1={y}
                  x2={width - padding}
                  y2={y}
                  stroke="#e0e0e0"
                  strokeWidth="1"
                />
                <text
                  x={padding - 10}
                  y={y + 5}
                  textAnchor="end"
                  fontSize="12"
                  fill="#666"
                >
                  {percent}%
                </text>
              </g>
            )
          })}

          {/* Chart line */}
          <polyline
            points={points}
            fill="none"
            stroke="#667eea"
            strokeWidth="3"
            strokeLinecap="round"
            strokeLinejoin="round"
          />

          {/* Fill area under line */}
          <polygon
            points={`${padding},${height - padding} ${points} ${width - padding},${height - padding}`}
            fill="url(#gradient)"
            opacity="0.3"
          />

          {/* Gradient definition */}
          <defs>
            <linearGradient id="gradient" x1="0%" y1="0%" x2="0%" y2="100%">
              <stop offset="0%" stopColor="#667eea" stopOpacity="0.8" />
              <stop offset="100%" stopColor="#667eea" stopOpacity="0.1" />
            </linearGradient>
          </defs>

          {/* Current value indicator */}
          {history.length > 0 && (
            <circle
              cx={padding + ((history.length - 1) / (history.length - 1)) * (width - 2 * padding)}
              cy={height - padding - ((currentPercentage - minPercentage) / (maxPercentage - minPercentage)) * (height - 2 * padding)}
              r="5"
              fill="#667eea"
            />
          )}

          {/* Time labels */}
          {firstTime && (
            <text
              x={padding}
              y={height - 10}
              textAnchor="start"
              fontSize="12"
              fill="#666"
            >
              {getTimeLabel(firstTime)}
            </text>
          )}
          {lastTime && (
            <text
              x={width - padding}
              y={height - 10}
              textAnchor="end"
              fontSize="12"
              fill="#666"
            >
              {getTimeLabel(lastTime)}
            </text>
          )}
        </svg>
      </div>

      <div className="chart-legend">
        <div className="legend-item">
          <span className="legend-color" style={{ background: '#667eea' }}></span>
          <span>Battery Percentage</span>
        </div>
        <div className="legend-stats">
          <span>Current: {Math.round(currentPercentage)}%</span>
          <span>Points: {history.length}</span>
        </div>
      </div>
    </div>
  )
}

export default BatteryChart
