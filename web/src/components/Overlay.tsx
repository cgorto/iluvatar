import { useStore } from '../store';
import { useEffect } from 'react';

export default function Overlay() {
  const { connected, objects, connect, disconnect } = useStore();
  
  // Auto-connect on mount
  useEffect(() => {
    connect();
    return () => disconnect();
  }, []);

  return (
    <div className="overlay-container">
      {/* Top Left: System Status */}
      <div className="hud-panel" style={{ width: '320px' }}>
        <h2 className="glitch-text" style={{ margin: 0, borderBottom: '1px solid var(--amber-dim)', paddingBottom: '0.5rem', letterSpacing: '2px' }}>
          ILUVATAR<span style={{ fontSize: '0.6em', verticalAlign: 'top' }}>V.0.9</span>
        </h2>
        
        <div style={{ marginTop: '1rem', display: 'flex', flexDirection: 'column', gap: '0.5rem', fontFamily: 'monospace' }}>
          <div style={{ display: 'flex', justifyContent: 'space-between' }}>
            <span>UPLINK_STATUS</span>
            <span style={{ color: connected ? '#00ff00' : '#ff0000', fontWeight: 'bold' }}>
              {connected ? ':: CONNECTED ::' : ':: OFFLINE ::'}
            </span>
          </div>
          
          <div style={{ display: 'flex', justifyContent: 'space-between' }}>
            <span>TRACKED_ENTITIES</span>
            <span>{objects.size.toString().padStart(3, '0')}</span>
          </div>
          
          <div style={{ display: 'flex', justifyContent: 'space-between' }}>
             <span>MEMORY_USAGE</span>
             <span>
               {/* @ts-ignore */}
               {((performance.memory?.usedJSHeapSize || 0) / 1048576).toFixed(1)} MB
             </span>
          </div>

          <button 
             onClick={connected ? disconnect : connect}
             style={{ 
               marginTop: '1rem', 
               background: connected ? 'rgba(255,0,0,0.1)' : 'rgba(255,176,0,0.1)', 
               border: `1px solid ${connected ? '#f00' : 'var(--text-main)'}`, 
               color: connected ? '#f00' : 'var(--text-main)', 
               padding: '0.8rem', 
               cursor: 'pointer',
               fontFamily: 'inherit',
               textTransform: 'uppercase',
               letterSpacing: '1px'
             }}
          >
             {connected ? 'TERMINATE LINK' : 'INITIATE HANDSHAKE'}
          </button>
        </div>
      </div>

      {/* Bottom Right: Object Table */}
      <div className="hud-panel" style={{ alignSelf: 'flex-end', width: '450px', maxHeight: '400px', overflowY: 'auto' }}>
         <h3 style={{ margin: 0, fontSize: '0.9rem', color: 'var(--amber-dim)', marginBottom: '0.5rem' }}>// ENTITY_MANIFEST_LOG</h3>
         <table style={{ width: '100%', fontSize: '0.8rem', borderCollapse: 'collapse' }}>
           <thead>
             <tr style={{ textAlign: 'left', color: '#777', borderBottom: '1px solid #333' }}>
               <th style={{ padding: '4px' }}>UID</th>
               <th style={{ padding: '4px' }}>COORDS [ENU]</th>
               <th style={{ padding: '4px' }}>CONF</th>
             </tr>
           </thead>
           <tbody>
             {Array.from(objects.values()).map(obj => (
               <tr key={obj.id.toString()} style={{ borderBottom: '1px solid #222' }}>
                 <td style={{ padding: '4px', color: '#aaa' }}>{obj.id.toString()}</td>
                 <td style={{ padding: '4px' }}>
                    {obj.centroid.x.toFixed(1)}/{obj.centroid.y.toFixed(1)}/{obj.centroid.z.toFixed(1)}
                 </td>
                 <td style={{ padding: '4px', color: obj.confidence > 0.8 ? '#0f0' : '#fa0' }}>
                    {(obj.confidence * 100).toFixed(0)}%
                 </td>
               </tr>
             ))}
             {objects.size === 0 && (
               <tr>
                 <td colSpan={3} style={{ padding: '1rem', textAlign: 'center', color: '#444', fontStyle: 'italic' }}>
                   NO SIGNAL DETECTED... SCANNING SECTOR...
                 </td>
               </tr>
             )}
           </tbody>
         </table>
      </div>
      
      {/* Decorative center crosshair */}
      <div style={{ position: 'absolute', top: '50%', left: '50%', transform: 'translate(-50%, -50%)', pointerEvents: 'none', opacity: 0.3 }}>
        <svg width="100" height="100" viewBox="0 0 100 100">
          <line x1="50" y1="20" x2="50" y2="80" stroke="var(--text-main)" strokeWidth="1" />
          <line x1="20" y1="50" x2="80" y2="50" stroke="var(--text-main)" strokeWidth="1" />
          <circle cx="50" cy="50" r="30" stroke="var(--text-main)" strokeWidth="1" fill="none" />
        </svg>
      </div>
      
      <div className="scanline"></div>
    </div>
  );
}
