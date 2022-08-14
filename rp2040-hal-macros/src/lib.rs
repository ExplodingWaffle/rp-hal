extern crate proc_macro;

use std::collections::HashSet;

use proc_macro::TokenStream;
use proc_macro2::Span;
use quote::quote;
use syn::{
    parse::{self, Parse, ParseStream, Result},
    parse_macro_input,
    punctuated::Punctuated,
    Ident, Item, ItemFn, Stmt, Token,
};

struct Args {
    vars: HashSet<Ident>,
}

impl Parse for Args {
    fn parse(input: ParseStream) -> Result<Self> {
        // parses comma seperated Idents
        let vars = Punctuated::<Ident, Token![,]>::parse_terminated(input)?;
        Ok(Args {
            vars: vars.into_iter().collect(),
        })
    }
}

#[proc_macro_attribute]
pub fn entry(args: TokenStream, input: TokenStream) -> TokenStream {
    let mut f = parse_macro_input!(input as ItemFn);
    let mut args = parse_macro_input!(args as Args);

    let clear_locks: TokenStream = quote!(unsafe {
        const SIO_BASE: u32 = 0xd0000000;
        const SPINLOCK0_PTR: *mut u32 = (SIO_BASE + 0x100) as *mut u32;
        const SPINLOCK_COUNT: usize = 32;
        for i in 0..SPINLOCK_COUNT {
            SPINLOCK0_PTR.wrapping_add(i).write_volatile(1);
        }
    })
    .into();
    let clear_locks = parse_macro_input!(clear_locks as Stmt);

    // statics must stay first so cortex_m_rt::entry still finds them
    let mut stmts = insert_after_static(f.block.stmts, clear_locks);

    if !args.vars.is_empty() {
        let flash_id_ident = Ident::new("flash_id", Span::call_site());
        if args.vars.take(&flash_id_ident).is_some() {
            let get_id: TokenStream = quote!(
                let flash_id: u64;
                unsafe {
                // Lookup function from bootrom
                let rom_table_lookup: unsafe extern "C" fn(*const u16, u32) = core::mem::transmute(*const u16 = 0x0000_0018 as _;);
                // Pointer to helper functions lookup table.
                const FUNC_TABLE: *const u16 = 0x0000_0014 as _;

                fn rom_func_lookup(code: u16) -> *const u32 {
                    rom_table_lookup(
                        FUNC_TABLE,
                        code as u32,
                    )
                }

                //boot2 is copied to ram so that we can use it once XIP is off
                let mut boot2 = [0u32; 256/4];
                core::ptr::copy_nonoverlapping(boot2.as_mut_ptr(), 0x10000000 as *const _, 64);

                let boot2_fn_ptr = (boot2 as *const u32 as *const u8).offset(1);
                let boot2_fn: unsafe extern "C" fn() -> () = core::mem::transmute(boot2_fn_ptr);

                #[repr(C)]
                struct FlashFunctionPointers<'a> {
                    connect_internal_flash: unsafe extern "C" fn() -> (),
                    flash_exit_xip: unsafe extern "C" fn() -> (),
                    flash_flush_cache: unsafe extern "C" fn() -> (),
                    flash_enter_xip: unsafe extern "C" fn() -> (),
                    phantom: core::marker::PhantomData<&'a ()>,
                }

                let ptrs = FlashFunctionPointers {
                    connect_internal_flash: core::mem::transmute(rom_func_lookup(u16::from_be_bytes([b'I', b'F']))),
                    flash_exit_xip: core::mem::transmute(rom_func_lookup(u16::from_be_bytes([b'E', b'X']))),
                    flash_flush_cache: core::mem::transmute(rom_func_lookup(u16::from_be_bytes([b'F', b'C']))),
                    flash_enter_xip: boot2_fn,
                    phantom: core::marker::PhantomData,
                };

                /*
                Should be equivalent to:
                rom_data::connect_internal_flash();
                rom_data::flash_exit_xip();
                send 4Bh command to get uid
                rom_data::flash_flush_cache();
                boot2();
                */
                /*core::arch::asm!(
                    "ldr r4, [{ptrs}, #0]",
                    "blx r4", // connect_internal_flash()

                    "ldr r4, [{ptrs}, #4]",
                    "blx r4", // flash_exit_xip()


                    "ldr r4, [{ptrs}, #12]",
                    "blx r4", // flash_flush_cache();

                    "ldr r4, [{ptrs}, #16]",
                    "blx r4", // boot2();
                    ptrs = in(reg) ptrs,
                    out("r0") flash_id,
                    out("r4") _,
                    clobber_abi("C"),
                );*/

                let flash_id: u64 = 0x6969696969696969;
                }
            ).into();
            let get_id = parse_macro_input!(get_id as Stmt);
            stmts = insert_after_static(stmts, get_id);
        }

        if !args.vars.is_empty() {
            return parse::Error::new(Span::call_site(), "Invalid arguments")
                .to_compile_error()
                .into();
        }
    }

    f.block.stmts = stmts;

    quote!(
        #[::cortex_m_rt::entry]
        #f
    )
    .into()
}

/// Insert new statements after initial block of statics
fn insert_after_static(stmts: impl IntoIterator<Item = Stmt>, insert: Stmt) -> Vec<Stmt> {
    let mut istmts = stmts.into_iter();
    let mut stmts = vec![];
    for stmt in istmts.by_ref() {
        match stmt {
            Stmt::Item(Item::Static(var)) => {
                stmts.push(Stmt::Item(Item::Static(var)));
            }
            _ => {
                stmts.push(insert);
                stmts.push(stmt);
                break;
            }
        }
    }
    stmts.extend(istmts);

    stmts
}
