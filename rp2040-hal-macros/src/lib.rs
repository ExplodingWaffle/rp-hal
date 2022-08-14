extern crate proc_macro;

use std::collections::HashSet;

use proc_macro::TokenStream;
use proc_macro2::Span;
use quote::quote;
use syn::{parse::{self, Parse, ParseStream, Result}, parse_macro_input, Item, ItemFn, Stmt, Ident, Token, punctuated::Punctuated};

struct Args{
    vars: HashSet<Ident>
}

impl Parse for Args{
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
                let flash_id: u64 = 0x6969696969696969;
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
