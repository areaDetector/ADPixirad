pro convert_pixirad_calibration, file

  xsize = 402
  ysize = 1024
  raw = fltarr(xsize, ysize)

  openr, lun, file+'.crrm', /get_lun
  readu, lun, raw
  close, lun
  write_tiff, file + '_crrm' + '.tif', /float, raw

  openr, lun, file+'.crrmnp', /get_lun
  readu, lun, raw
  close, lun
  write_tiff, file + '_crrmnp' + '.tif', /float, raw
  
  openr, lun, file+'.crrmnps', /get_lun
  readu, lun, raw
  close, lun
  write_tiff, file + '_crrmnps' + '.tif', /float, raw
  
  
end  
