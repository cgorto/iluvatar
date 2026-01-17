import { forwardRef, useMemo } from 'react'
import { DitherEffect } from './DitherEffect'
import { CRTDistortion } from './CRTDistortion'

export const Dither = forwardRef(({ scale = 1.0 }: { scale?: number }, ref) => {
  const effect = useMemo(() => new DitherEffect({ scale }), [scale])
  return <primitive ref={ref} object={effect} dispose={null} />
})

export const CRT = forwardRef(({ curvature = 1.0, aberration = 1.0, vignette = 1.5 }: any, ref) => {
    const effect = useMemo(() => new CRTDistortion({ curvature, aberration, vignette }), [curvature, aberration, vignette])
    return <primitive ref={ref} object={effect} dispose={null} />
})
