 .386P
 .model FLAT
 externdef _d_zistepu:dword
 externdef _d_pzbuffer:dword
 externdef _d_zistepv:dword
 externdef _d_zrowbytes:dword
 externdef _d_ziorigin:dword
 externdef _r_turb_s:dword
 externdef _r_turb_t:dword
 externdef _r_turb_pdest:dword
 externdef _r_turb_spancount:dword
 externdef _r_turb_turb:dword
 externdef _r_turb_pbase:dword
 externdef _r_turb_sstep:dword
 externdef _r_turb_tstep:dword
 externdef _r_bmodelactive:dword
 externdef _d_sdivzstepu:dword
 externdef _d_tdivzstepu:dword
 externdef _d_sdivzstepv:dword
 externdef _d_tdivzstepv:dword
 externdef _d_sdivzorigin:dword
 externdef _d_tdivzorigin:dword
 externdef _sadjust:dword
 externdef _tadjust:dword
 externdef _bbextents:dword
 externdef _bbextentt:dword
 externdef _cacheblock:dword
 externdef _d_viewbuffer:dword
 externdef _cachewidth:dword
 externdef _d_pzbuffer:dword
 externdef _d_zrowbytes:dword
 externdef _d_zwidth:dword
 externdef _d_scantable:dword
 externdef _r_lightptr:dword
 externdef _r_numvblocks:dword
 externdef _prowdestbase:dword
 externdef _pbasesource:dword
 externdef _r_lightwidth:dword
 externdef _lightright:dword
 externdef _lightrightstep:dword
 externdef _lightdeltastep:dword
 externdef _lightdelta:dword
 externdef _lightright:dword
 externdef _lightdelta:dword
 externdef _sourcetstep:dword
 externdef _surfrowbytes:dword
 externdef _lightrightstep:dword
 externdef _lightdeltastep:dword
 externdef _r_sourcemax:dword
 externdef _r_stepback:dword
 externdef _colormap:dword
 externdef _blocksize:dword
 externdef _sourcesstep:dword
 externdef _lightleft:dword
 externdef _blockdivshift:dword
 externdef _blockdivmask:dword
 externdef _lightleftstep:dword
 externdef _r_origin:dword
 externdef _r_ppn:dword
 externdef _r_pup:dword
 externdef _r_pright:dword
 externdef _ycenter:dword
 externdef _xcenter:dword
 externdef _d_vrectbottom_particle:dword
 externdef _d_vrectright_particle:dword
 externdef _d_vrecty:dword
 externdef _d_vrectx:dword
 externdef _d_pix_shift:dword
 externdef _d_pix_min:dword
 externdef _d_pix_max:dword
 externdef _d_y_aspect_shift:dword
 externdef _screenwidth:dword
 externdef _r_leftclipped:dword
 externdef _r_leftenter:dword
 externdef _r_rightclipped:dword
 externdef _r_rightenter:dword
 externdef _modelorg:dword
 externdef _xscale:dword
 externdef _r_refdef:dword
 externdef _yscale:dword
 externdef _r_leftexit:dword
 externdef _r_rightexit:dword
 externdef _r_lastvertvalid:dword
 externdef _cacheoffset:dword
 externdef _newedges:dword
 externdef _removeedges:dword
 externdef _r_pedge:dword
 externdef _r_framecount:dword
 externdef _r_u1:dword
 externdef _r_emitted:dword
 externdef _edge_p:dword
 externdef _surface_p:dword
 externdef _surfaces:dword
 externdef _r_lzi1:dword
 externdef _r_v1:dword
 externdef _r_ceilv1:dword
 externdef _r_nearzi:dword
 externdef _r_nearzionly:dword
 externdef _edge_aftertail:dword
 externdef _edge_tail:dword
 externdef _current_iv:dword
 externdef _edge_head_u_shift20:dword
 externdef _span_p:dword
 externdef _edge_head:dword
 externdef _fv:dword
 externdef _edge_tail_u_shift20:dword
 externdef _r_apverts:dword
 externdef _r_anumverts:dword
 externdef _aliastransform:dword
 externdef _r_avertexnormals:dword
 externdef _r_plightvec:dword
 externdef _r_ambientlight:dword
 externdef _r_shadelight:dword
 externdef _aliasxcenter:dword
 externdef _aliasycenter:dword
 externdef _a_sstepxfrac:dword
 externdef _r_affinetridesc:dword
 externdef _acolormap:dword
 externdef _d_pcolormap:dword
 externdef _r_affinetridesc:dword
 externdef _d_sfrac:dword
 externdef _d_ptex:dword
 externdef _d_pedgespanpackage:dword
 externdef _d_tfrac:dword
 externdef _d_light:dword
 externdef _d_zi:dword
 externdef _d_pdest:dword
 externdef _d_pz:dword
 externdef _d_aspancount:dword
 externdef _erroradjustup:dword
 externdef _errorterm:dword
 externdef _d_xdenom:dword
 externdef _r_p0:dword
 externdef _r_p1:dword
 externdef _r_p2:dword
 externdef _a_tstepxfrac:dword
 externdef _r_sstepx:dword
 externdef _r_tstepx:dword
 externdef _a_ststepxwhole:dword
 externdef _zspantable:dword
 externdef _skintable:dword
 externdef _r_zistepx:dword
 externdef _erroradjustdown:dword
 externdef _d_countextrastep:dword
 externdef _ubasestep:dword
 externdef _a_ststepxwhole:dword
 externdef _a_tstepxfrac:dword
 externdef _r_lstepx:dword
 externdef _a_spans:dword
 externdef _erroradjustdown:dword
 externdef _d_pdestextrastep:dword
 externdef _d_pzextrastep:dword
 externdef _d_sfracextrastep:dword
 externdef _d_ptexextrastep:dword
 externdef _d_countextrastep:dword
 externdef _d_tfracextrastep:dword
 externdef _d_lightextrastep:dword
 externdef _d_ziextrastep:dword
 externdef _d_pdestbasestep:dword
 externdef _d_pzbasestep:dword
 externdef _d_sfracbasestep:dword
 externdef _d_ptexbasestep:dword
 externdef _ubasestep:dword
 externdef _d_tfracbasestep:dword
 externdef _d_lightbasestep:dword
 externdef _d_zibasestep:dword
 externdef _zspantable:dword
 externdef _r_lstepy:dword
 externdef _r_sstepy:dword
 externdef _r_tstepy:dword
 externdef _r_zistepy:dword
 externdef _D_PolysetSetEdgeTable:dword
 externdef _D_RasterizeAliasPolySmooth:dword
 externdef float_point5:dword
 externdef Float2ToThe31nd:dword
 externdef izistep:dword
 externdef izi:dword
 externdef FloatMinus2ToThe31nd:dword
 externdef float_1:dword
 externdef float_particle_z_clip:dword
 externdef float_minus_1:dword
 externdef float_0:dword
 externdef fp_16:dword
 externdef fp_64k:dword
 externdef fp_1m:dword
 externdef fp_1m_minus_1:dword
 externdef fp_8:dword
 externdef entryvec_table:dword
 externdef advancetable:dword
 externdef sstep:dword
 externdef tstep:dword
 externdef pspantemp:dword
 externdef counttemp:dword
 externdef jumptemp:dword
 externdef reciprocal_table:dword
 externdef DP_Count:dword
 externdef DP_u:dword
 externdef DP_v:dword
 externdef DP_32768:dword
 externdef DP_Color:dword
 externdef DP_Pix:dword
 externdef DP_EntryTable:dword
 externdef pbase:dword
 externdef s:dword
 externdef t:dword
 externdef sfracf:dword
 externdef tfracf:dword
 externdef snext:dword
 externdef tnext:dword
 externdef spancountminus1:dword
 externdef zi16stepu:dword
 externdef sdivz16stepu:dword
 externdef tdivz16stepu:dword
 externdef zi8stepu:dword
 externdef sdivz8stepu:dword
 externdef tdivz8stepu:dword
 externdef reciprocal_table_16:dword
 externdef entryvec_table_16:dword
 externdef ceil_cw:dword
 externdef single_cw:dword
 externdef fp_64kx64k:dword
 externdef pz:dword
 externdef spr8entryvec_table:dword
 externdef _snd_scaletable:dword
 externdef _paintbuffer:dword
 externdef _snd_linear_count:dword
 externdef _snd_p:dword
 externdef _snd_vol:dword
 externdef _snd_out:dword
 externdef _vright:dword
 externdef _vup:dword
 externdef _vpn:dword
 externdef _BOPS_Error:dword
_DATA SEGMENT
sb_v dd 0
_DATA ENDS
_TEXT SEGMENT
 align 4
 public _R_Surf8Start
_R_Surf8Start:
 align 4
 public _R_DrawSurfaceBlock8_mip0
_R_DrawSurfaceBlock8_mip0:
 push ebp
 push edi
 push esi
 push ebx
 mov ebx,ds:dword ptr[_r_lightptr]
 mov eax,ds:dword ptr[_r_numvblocks]
 mov ds:dword ptr[sb_v],eax
 mov edi,ds:dword ptr[_prowdestbase]
 mov esi,ds:dword ptr[_pbasesource]
Lv_loop_mip0:
 mov eax,ds:dword ptr[ebx]
 mov edx,ds:dword ptr[4+ebx]
 mov ebp,eax
 mov ecx,ds:dword ptr[_r_lightwidth]
 mov ds:dword ptr[_lightright],edx
 sub ebp,edx
 and ebp,0FFFFFh
 lea ebx,ds:dword ptr[ebx+ecx*4]
 mov ds:dword ptr[_r_lightptr],ebx
 mov ecx,ds:dword ptr[4+ebx]
 mov ebx,ds:dword ptr[ebx]
 sub ebx,eax
 sub ecx,edx
 sar ecx,4
 or ebp,0F0000000h
 sar ebx,4
 mov ds:dword ptr[_lightrightstep],ecx
 sub ebx,ecx
 and ebx,0FFFFFh
 or ebx,0F0000000h
 sub ecx,ecx
 mov ds:dword ptr[_lightdeltastep],ebx
 sub ebx,ebx
Lblockloop8_mip0:
 mov ds:dword ptr[_lightdelta],ebp
 mov cl,ds:byte ptr[14+esi]
 sar ebp,4
 mov bh,dh
 mov bl,ds:byte ptr[15+esi]
 add edx,ebp
 mov ch,dh
 add edx,ebp
 mov ah,ds:byte ptr[12345678h+ebx]
LBPatch0:
 mov bl,ds:byte ptr[13+esi]
 mov al,ds:byte ptr[12345678h+ecx]
LBPatch1:
 mov cl,ds:byte ptr[12+esi]
 mov bh,dh
 add edx,ebp
 ror eax,16
 mov ch,dh
 add edx,ebp
 mov ah,ds:byte ptr[12345678h+ebx]
LBPatch2:
 mov bl,ds:byte ptr[11+esi]
 mov al,ds:byte ptr[12345678h+ecx]
LBPatch3:
 mov cl,ds:byte ptr[10+esi]
 mov ds:dword ptr[12+edi],eax
 mov bh,dh
 add edx,ebp
 mov ch,dh
 add edx,ebp
 mov ah,ds:byte ptr[12345678h+ebx]
LBPatch4:
 mov bl,ds:byte ptr[9+esi]
 mov al,ds:byte ptr[12345678h+ecx]
LBPatch5:
 mov cl,ds:byte ptr[8+esi]
 mov bh,dh
 add edx,ebp
 ror eax,16
 mov ch,dh
 add edx,ebp
 mov ah,ds:byte ptr[12345678h+ebx]
LBPatch6:
 mov bl,ds:byte ptr[7+esi]
 mov al,ds:byte ptr[12345678h+ecx]
LBPatch7:
 mov cl,ds:byte ptr[6+esi]
 mov ds:dword ptr[8+edi],eax
 mov bh,dh
 add edx,ebp
 mov ch,dh
 add edx,ebp
 mov ah,ds:byte ptr[12345678h+ebx]
LBPatch8:
 mov bl,ds:byte ptr[5+esi]
 mov al,ds:byte ptr[12345678h+ecx]
LBPatch9:
 mov cl,ds:byte ptr[4+esi]
 mov bh,dh
 add edx,ebp
 ror eax,16
 mov ch,dh
 add edx,ebp
 mov ah,ds:byte ptr[12345678h+ebx]
LBPatch10:
 mov bl,ds:byte ptr[3+esi]
 mov al,ds:byte ptr[12345678h+ecx]
LBPatch11:
 mov cl,ds:byte ptr[2+esi]
 mov ds:dword ptr[4+edi],eax
 mov bh,dh
 add edx,ebp
 mov ch,dh
 add edx,ebp
 mov ah,ds:byte ptr[12345678h+ebx]
LBPatch12:
 mov bl,ds:byte ptr[1+esi]
 mov al,ds:byte ptr[12345678h+ecx]
LBPatch13:
 mov cl,ds:byte ptr[esi]
 mov bh,dh
 add edx,ebp
 ror eax,16
 mov ch,dh
 mov ah,ds:byte ptr[12345678h+ebx]
LBPatch14:
 mov edx,ds:dword ptr[_lightright]
 mov al,ds:byte ptr[12345678h+ecx]
LBPatch15:
 mov ebp,ds:dword ptr[_lightdelta]
 mov ds:dword ptr[edi],eax
 add esi,ds:dword ptr[_sourcetstep]
 add edi,ds:dword ptr[_surfrowbytes]
 add edx,ds:dword ptr[_lightrightstep]
 add ebp,ds:dword ptr[_lightdeltastep]
 mov ds:dword ptr[_lightright],edx
 jc Lblockloop8_mip0
 cmp esi,ds:dword ptr[_r_sourcemax]
 jb LSkip_mip0
 sub esi,ds:dword ptr[_r_stepback]
LSkip_mip0:
 mov ebx,ds:dword ptr[_r_lightptr]
 dec ds:dword ptr[sb_v]
 jnz Lv_loop_mip0
 pop ebx
 pop esi
 pop edi
 pop ebp
 ret
 align 4
 public _R_DrawSurfaceBlock8_mip1
_R_DrawSurfaceBlock8_mip1:
 push ebp
 push edi
 push esi
 push ebx
 mov ebx,ds:dword ptr[_r_lightptr]
 mov eax,ds:dword ptr[_r_numvblocks]
 mov ds:dword ptr[sb_v],eax
 mov edi,ds:dword ptr[_prowdestbase]
 mov esi,ds:dword ptr[_pbasesource]
Lv_loop_mip1:
 mov eax,ds:dword ptr[ebx]
 mov edx,ds:dword ptr[4+ebx]
 mov ebp,eax
 mov ecx,ds:dword ptr[_r_lightwidth]
 mov ds:dword ptr[_lightright],edx
 sub ebp,edx
 and ebp,0FFFFFh
 lea ebx,ds:dword ptr[ebx+ecx*4]
 mov ds:dword ptr[_r_lightptr],ebx
 mov ecx,ds:dword ptr[4+ebx]
 mov ebx,ds:dword ptr[ebx]
 sub ebx,eax
 sub ecx,edx
 sar ecx,3
 or ebp,070000000h
 sar ebx,3
 mov ds:dword ptr[_lightrightstep],ecx
 sub ebx,ecx
 and ebx,0FFFFFh
 or ebx,0F0000000h
 sub ecx,ecx
 mov ds:dword ptr[_lightdeltastep],ebx
 sub ebx,ebx
Lblockloop8_mip1:
 mov ds:dword ptr[_lightdelta],ebp
 mov cl,ds:byte ptr[6+esi]
 sar ebp,3
 mov bh,dh
 mov bl,ds:byte ptr[7+esi]
 add edx,ebp
 mov ch,dh
 add edx,ebp
 mov ah,ds:byte ptr[12345678h+ebx]
LBPatch22:
 mov bl,ds:byte ptr[5+esi]
 mov al,ds:byte ptr[12345678h+ecx]
LBPatch23:
 mov cl,ds:byte ptr[4+esi]
 mov bh,dh
 add edx,ebp
 ror eax,16
 mov ch,dh
 add edx,ebp
 mov ah,ds:byte ptr[12345678h+ebx]
LBPatch24:
 mov bl,ds:byte ptr[3+esi]
 mov al,ds:byte ptr[12345678h+ecx]
LBPatch25:
 mov cl,ds:byte ptr[2+esi]
 mov ds:dword ptr[4+edi],eax
 mov bh,dh
 add edx,ebp
 mov ch,dh
 add edx,ebp
 mov ah,ds:byte ptr[12345678h+ebx]
LBPatch26:
 mov bl,ds:byte ptr[1+esi]
 mov al,ds:byte ptr[12345678h+ecx]
LBPatch27:
 mov cl,ds:byte ptr[esi]
 mov bh,dh
 add edx,ebp
 ror eax,16
 mov ch,dh
 mov ah,ds:byte ptr[12345678h+ebx]
LBPatch28:
 mov edx,ds:dword ptr[_lightright]
 mov al,ds:byte ptr[12345678h+ecx]
LBPatch29:
 mov ebp,ds:dword ptr[_lightdelta]
 mov ds:dword ptr[edi],eax
 mov eax,ds:dword ptr[_sourcetstep]
 add esi,eax
 mov eax,ds:dword ptr[_surfrowbytes]
 add edi,eax
 mov eax,ds:dword ptr[_lightrightstep]
 add edx,eax
 mov eax,ds:dword ptr[_lightdeltastep]
 add ebp,eax
 mov ds:dword ptr[_lightright],edx
 jc Lblockloop8_mip1
 cmp esi,ds:dword ptr[_r_sourcemax]
 jb LSkip_mip1
 sub esi,ds:dword ptr[_r_stepback]
LSkip_mip1:
 mov ebx,ds:dword ptr[_r_lightptr]
 dec ds:dword ptr[sb_v]
 jnz Lv_loop_mip1
 pop ebx
 pop esi
 pop edi
 pop ebp
 ret
 align 4
 public _R_DrawSurfaceBlock8_mip2
_R_DrawSurfaceBlock8_mip2:
 push ebp
 push edi
 push esi
 push ebx
 mov ebx,ds:dword ptr[_r_lightptr]
 mov eax,ds:dword ptr[_r_numvblocks]
 mov ds:dword ptr[sb_v],eax
 mov edi,ds:dword ptr[_prowdestbase]
 mov esi,ds:dword ptr[_pbasesource]
Lv_loop_mip2:
 mov eax,ds:dword ptr[ebx]
 mov edx,ds:dword ptr[4+ebx]
 mov ebp,eax
 mov ecx,ds:dword ptr[_r_lightwidth]
 mov ds:dword ptr[_lightright],edx
 sub ebp,edx
 and ebp,0FFFFFh
 lea ebx,ds:dword ptr[ebx+ecx*4]
 mov ds:dword ptr[_r_lightptr],ebx
 mov ecx,ds:dword ptr[4+ebx]
 mov ebx,ds:dword ptr[ebx]
 sub ebx,eax
 sub ecx,edx
 sar ecx,2
 or ebp,030000000h
 sar ebx,2
 mov ds:dword ptr[_lightrightstep],ecx
 sub ebx,ecx
 and ebx,0FFFFFh
 or ebx,0F0000000h
 sub ecx,ecx
 mov ds:dword ptr[_lightdeltastep],ebx
 sub ebx,ebx
Lblockloop8_mip2:
 mov ds:dword ptr[_lightdelta],ebp
 mov cl,ds:byte ptr[2+esi]
 sar ebp,2
 mov bh,dh
 mov bl,ds:byte ptr[3+esi]
 add edx,ebp
 mov ch,dh
 add edx,ebp
 mov ah,ds:byte ptr[12345678h+ebx]
LBPatch18:
 mov bl,ds:byte ptr[1+esi]
 mov al,ds:byte ptr[12345678h+ecx]
LBPatch19:
 mov cl,ds:byte ptr[esi]
 mov bh,dh
 add edx,ebp
 ror eax,16
 mov ch,dh
 mov ah,ds:byte ptr[12345678h+ebx]
LBPatch20:
 mov edx,ds:dword ptr[_lightright]
 mov al,ds:byte ptr[12345678h+ecx]
LBPatch21:
 mov ebp,ds:dword ptr[_lightdelta]
 mov ds:dword ptr[edi],eax
 mov eax,ds:dword ptr[_sourcetstep]
 add esi,eax
 mov eax,ds:dword ptr[_surfrowbytes]
 add edi,eax
 mov eax,ds:dword ptr[_lightrightstep]
 add edx,eax
 mov eax,ds:dword ptr[_lightdeltastep]
 add ebp,eax
 mov ds:dword ptr[_lightright],edx
 jc Lblockloop8_mip2
 cmp esi,ds:dword ptr[_r_sourcemax]
 jb LSkip_mip2
 sub esi,ds:dword ptr[_r_stepback]
LSkip_mip2:
 mov ebx,ds:dword ptr[_r_lightptr]
 dec ds:dword ptr[sb_v]
 jnz Lv_loop_mip2
 pop ebx
 pop esi
 pop edi
 pop ebp
 ret
 align 4
 public _R_DrawSurfaceBlock8_mip3
_R_DrawSurfaceBlock8_mip3:
 push ebp
 push edi
 push esi
 push ebx
 mov ebx,ds:dword ptr[_r_lightptr]
 mov eax,ds:dword ptr[_r_numvblocks]
 mov ds:dword ptr[sb_v],eax
 mov edi,ds:dword ptr[_prowdestbase]
 mov esi,ds:dword ptr[_pbasesource]
Lv_loop_mip3:
 mov eax,ds:dword ptr[ebx]
 mov edx,ds:dword ptr[4+ebx]
 mov ebp,eax
 mov ecx,ds:dword ptr[_r_lightwidth]
 mov ds:dword ptr[_lightright],edx
 sub ebp,edx
 and ebp,0FFFFFh
 lea ebx,ds:dword ptr[ebx+ecx*4]
 mov ds:dword ptr[_lightdelta],ebp
 mov ds:dword ptr[_r_lightptr],ebx
 mov ecx,ds:dword ptr[4+ebx]
 mov ebx,ds:dword ptr[ebx]
 sub ebx,eax
 sub ecx,edx
 sar ecx,1
 sar ebx,1
 mov ds:dword ptr[_lightrightstep],ecx
 sub ebx,ecx
 and ebx,0FFFFFh
 sar ebp,1
 or ebx,0F0000000h
 mov ds:dword ptr[_lightdeltastep],ebx
 sub ebx,ebx
 mov bl,ds:byte ptr[1+esi]
 sub ecx,ecx
 mov bh,dh
 mov cl,ds:byte ptr[esi]
 add edx,ebp
 mov ch,dh
 mov al,ds:byte ptr[12345678h+ebx]
LBPatch16:
 mov edx,ds:dword ptr[_lightright]
 mov ds:byte ptr[1+edi],al
 mov al,ds:byte ptr[12345678h+ecx]
LBPatch17:
 mov ds:byte ptr[edi],al
 mov eax,ds:dword ptr[_sourcetstep]
 add esi,eax
 mov eax,ds:dword ptr[_surfrowbytes]
 add edi,eax
 mov eax,ds:dword ptr[_lightdeltastep]
 mov ebp,ds:dword ptr[_lightdelta]
 mov cl,ds:byte ptr[esi]
 add ebp,eax
 mov eax,ds:dword ptr[_lightrightstep]
 sar ebp,1
 add edx,eax
 mov bh,dh
 mov bl,ds:byte ptr[1+esi]
 add edx,ebp
 mov ch,dh
 mov al,ds:byte ptr[12345678h+ebx]
LBPatch30:
 mov edx,ds:dword ptr[_sourcetstep]
 mov ds:byte ptr[1+edi],al
 mov al,ds:byte ptr[12345678h+ecx]
LBPatch31:
 mov ds:byte ptr[edi],al
 mov ebp,ds:dword ptr[_surfrowbytes]
 add esi,edx
 add edi,ebp
 cmp esi,ds:dword ptr[_r_sourcemax]
 jb LSkip_mip3
 sub esi,ds:dword ptr[_r_stepback]
LSkip_mip3:
 mov ebx,ds:dword ptr[_r_lightptr]
 dec ds:dword ptr[sb_v]
 jnz Lv_loop_mip3
 pop ebx
 pop esi
 pop edi
 pop ebp
 ret
 public _R_Surf8End
_R_Surf8End:
_TEXT ENDS
_DATA SEGMENT
 align 4
LPatchTable8:
 dd LBPatch0-4
 dd LBPatch1-4
 dd LBPatch2-4
 dd LBPatch3-4
 dd LBPatch4-4
 dd LBPatch5-4
 dd LBPatch6-4
 dd LBPatch7-4
 dd LBPatch8-4
 dd LBPatch9-4
 dd LBPatch10-4
 dd LBPatch11-4
 dd LBPatch12-4
 dd LBPatch13-4
 dd LBPatch14-4
 dd LBPatch15-4
 dd LBPatch16-4
 dd LBPatch17-4
 dd LBPatch18-4
 dd LBPatch19-4
 dd LBPatch20-4
 dd LBPatch21-4
 dd LBPatch22-4
 dd LBPatch23-4
 dd LBPatch24-4
 dd LBPatch25-4
 dd LBPatch26-4
 dd LBPatch27-4
 dd LBPatch28-4
 dd LBPatch29-4
 dd LBPatch30-4
 dd LBPatch31-4
_DATA ENDS
_TEXT SEGMENT
 align 4
 public _R_Surf8Patch
_R_Surf8Patch:
 push ebx
 mov eax,ds:dword ptr[_colormap]
 mov ebx,offset LPatchTable8
 mov ecx,32
LPatchLoop8:
 mov edx,ds:dword ptr[ebx]
 add ebx,4
 mov ds:dword ptr[edx],eax
 dec ecx
 jnz LPatchLoop8
 pop ebx
 ret
_TEXT ENDS
 END
