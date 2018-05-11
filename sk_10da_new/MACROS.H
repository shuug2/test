/*            - macros.h -

   This file #defines frequently used instructions as macros.
   Used with ICCA90.

   File version:   $Revision: 1.0

   Usage, i.e.:
          INWORD(TCNT1,i);       // Reads the counter value
          OUTWORD(TCNT1,i);      // writes the counter value        
*/

/* Reads 16-bit register */
#define OUTWORD(ADDRESS,VAL)\
{\
  cli();\
  ADDRESS = VAL;\
  sei();\
}

/* Writes 16-bit register */
#define INWORD(ADDRESS,VAL)\
{\
  cli();\
  VAL = ADDRESS;\
  sei();\
}

/* Set bit in I/O register - command is resulting in SBI assembly instruction */
#define sbi(ADDRESS,BIT) (ADDRESS |= _BV(BIT))

/* Clear bit in I/O regsiter - command is resulting in CBI assembly instruction */
#define cbi(ADDRESS,BIT)  (ADDRESS &= ~_BV(BIT))

/* Test bit in I/O register */
#define tbi(ADDRESS,BIT)  (ADDRESS & _BV(BIT))

