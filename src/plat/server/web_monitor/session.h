/*
 * session.h
 *
 *  Created on: Nov 07, 2013
 *      Author: uplusplus
 */
#include "mongoose.h"

#define authorize_url  "/authorize"
#define login_url      "/login.html"

// Redirect user to the login form. In the cookie, store the original URL
// we came from, so that after the authorization we could redirect back.
void redirect_to_login(struct mg_connection *conn,
		const struct mg_request_info *request_info);

// A handler for the /authorize endpoint.
// Login page form sends user name and password to this endpoint.
void authorize(struct mg_connection *conn,
		const struct mg_request_info *request_info);

// Return 1 if request is authorized, 0 otherwise.
int is_authorized(const struct mg_connection *conn,
		const struct mg_request_info *request_info);
