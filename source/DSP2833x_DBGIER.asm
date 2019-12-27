;// TI File $Revision: /main/1 $
;// Checkin $Date: August 18, 2006   13:46:03 $
;//###########################################################################
;//
;// FILE:  DSP2833x_DBGIER.asm
;//
;// TITLE: Set the DBGIER register
;//
;// DESCRIPTION:
;//  
;//  Function to set the DBGIER register (for realtime emulation).
;//  Function Prototype: void SetDBGIER(Uint16)
;//  Useage: SetDBGIER(value);
;//  Input Parameters: Uint16 value = value to put in DBGIER register. 
;//  Return Value: none          
;//
;//###########################################################################
;// $TI Release: 2833x/2823x Header Files and Peripheral Examples V133 $
;// $Release Date: June 8, 2012 $
;//###########################################################################	
		.global _SetDBGIER
		.text
		
_SetDBGIER:
		MOV 	*SP++,AL
		POP 	DBGIER
		LRETR
		
